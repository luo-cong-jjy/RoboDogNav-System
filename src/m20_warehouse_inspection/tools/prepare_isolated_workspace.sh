#!/usr/bin/env bash
#
# Prepare a source-only M20 warehouse workspace from pinned dependencies.
# The target must be empty; this script never removes or overwrites a workspace.

set -euo pipefail

usage() {
  echo "Usage: $0 TARGET_WORKSPACE [--project-src DIR] [--scan-repository REPO] [--model-repository REPO] [--sdk-repository REPO]"
}

if [[ $# -lt 1 ]]; then
  usage
  exit 2
fi

target_workspace="$1"
shift
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
integration_source="$(cd -- "${script_dir}/.." && pwd)"
project_source="$(cd -- "${integration_source}/.." && pwd)"
scan_repository=""
model_repository=""
sdk_repository=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --project-src)
      project_source="$2"
      shift 2
      ;;
    --scan-repository)
      scan_repository="$2"
      shift 2
      ;;
    --model-repository)
      model_repository="$2"
      shift 2
      ;;
    --sdk-repository)
      sdk_repository="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

target_workspace="$(realpath -m -- "${target_workspace}")"
project_source="$(realpath -- "${project_source}")"
if [[ "${target_workspace}" == "/" || "${target_workspace}" == "${project_source}" ]]; then
  echo "Refusing unsafe target workspace: ${target_workspace}" >&2
  exit 2
fi
if [[ -e "${target_workspace}" ]] && [[ -n "$(find "${target_workspace}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
  echo "Target workspace must be empty: ${target_workspace}" >&2
  exit 2
fi

required_commands=(git realpath rsync sha256sum)
for command_name in "${required_commands[@]}"; do
  command -v "${command_name}" >/dev/null || {
    echo "Required command is unavailable: ${command_name}" >&2
    exit 3
  }
done

packages=(
  m20_warehouse_interfaces
  m20_multifloor_map
  m20_official_description
  m20_warehouse_sim
  m20_inspection_core
  m20_scan_planner
  m20_scan_navigation
  m20_locomotion_control
  m20_mujoco_backend
  m20_warehouse_inspection
)
for package_name in "${packages[@]}"; do
  if [[ ! -f "${project_source}/${package_name}/package.xml" ]]; then
    echo "Missing project package: ${project_source}/${package_name}" >&2
    exit 4
  fi
done

mkdir -p "${target_workspace}/src"
for package_name in "${packages[@]}"; do
  rsync -a \
    --exclude '__pycache__/' \
    --exclude '.pytest_cache/' \
    --exclude '*.pyc' \
    "${project_source}/${package_name}/" \
    "${target_workspace}/src/${package_name}/"
done

manifest="${target_workspace}/src/m20_warehouse_inspection/dependencies.repos"
sdk_manifest="${target_workspace}/src/m20_warehouse_inspection/dependencies_sdk.repos"
lock_file="${target_workspace}/src/m20_warehouse_inspection/config/workspace_lock.yaml"
patch_file="${target_workspace}/src/m20_warehouse_inspection/patches/scan_planner_humble_cpu.patch"
integration_patch_file="${target_workspace}/src/m20_warehouse_inspection/patches/scan_planner_multifloor_integration.patch"
sdk_patch_file="${target_workspace}/src/m20_warehouse_inspection/patches/m20_sdk_cmdvel_adapter.patch"
scan_revision="d0b921c9b05a6d291d144d60882b2e0e88d2c0e0"
model_revision="6113c62da96295e8d53abbc079af5296bf4649f8"
sdk_revision="ee289d475f2dedf0332b7542f2b173fa9e8d1456"
scan_target="${target_workspace}/src/vendor/SCAN-Planner"
model_target="${target_workspace}/src/vendor/deep_robotics_model"
sdk_target="${target_workspace}/src/third_party/sdk_deploy"

if [[ -n "${scan_repository}" || -n "${model_repository}" || -n "${sdk_repository}" ]]; then
  if [[ -z "${scan_repository}" || -z "${model_repository}" || -z "${sdk_repository}" ]]; then
    echo "All three local dependency repositories must be supplied together" >&2
    exit 5
  fi
  mkdir -p "${target_workspace}/src/vendor" "${target_workspace}/src/third_party"
  git clone --quiet --no-hardlinks "${scan_repository}" "${scan_target}"
  git -C "${scan_target}" checkout --quiet --detach "${scan_revision}"
  git clone --quiet --no-hardlinks "${model_repository}" "${model_target}"
  git -C "${model_target}" checkout --quiet --detach "${model_revision}"
  git clone --quiet --no-hardlinks "${sdk_repository}" "${sdk_target}"
  git -C "${sdk_target}" checkout --quiet --detach "${sdk_revision}"
else
  command -v vcs >/dev/null || {
    echo "vcs is required when local dependency repositories are not supplied" >&2
    exit 3
  }
  (
    cd -- "${target_workspace}"
    vcs import . < "${manifest}"
    vcs import . < "${sdk_manifest}"
  )
fi

actual_scan_revision="$(git -C "${scan_target}" rev-parse HEAD)"
actual_model_revision="$(git -C "${model_target}" rev-parse HEAD)"
actual_sdk_revision="$(git -C "${sdk_target}" rev-parse HEAD)"
if [[ "${actual_scan_revision}" != "${scan_revision}" ]]; then
  echo "Unexpected SCAN-Planner revision: ${actual_scan_revision}" >&2
  exit 6
fi
if [[ "${actual_model_revision}" != "${model_revision}" ]]; then
  echo "Unexpected model revision: ${actual_model_revision}" >&2
  exit 6
fi
if [[ "${actual_sdk_revision}" != "${sdk_revision}" ]]; then
  echo "Unexpected M20 SDK revision: ${actual_sdk_revision}" >&2
  exit 6
fi

patch_sha256="$(sha256sum "${patch_file}" | awk '{print $1}')"
lock_patch_sha256="$(sed -n 's/^[[:space:]]*patch_sha256:[[:space:]]*//p' "${lock_file}")"
if [[ "${patch_sha256}" != "${lock_patch_sha256}" ]]; then
  echo "Dependency patch checksum mismatch" >&2
  echo "expected=${lock_patch_sha256} actual=${patch_sha256}" >&2
  exit 7
fi
integration_patch_sha256="$(sha256sum "${integration_patch_file}" | awk '{print $1}')"
lock_integration_patch_sha256="$(
  sed -n 's/^[[:space:]]*integration_patch_sha256:[[:space:]]*//p' "${lock_file}"
)"
if [[ "${integration_patch_sha256}" != "${lock_integration_patch_sha256}" ]]; then
  echo "Integration patch checksum mismatch" >&2
  echo "expected=${lock_integration_patch_sha256} actual=${integration_patch_sha256}" >&2
  exit 7
fi
sdk_patch_sha256="$(sha256sum "${sdk_patch_file}" | awk '{print $1}')"
lock_sdk_patch_sha256="$(
  sed -n 's/^[[:space:]]*sdk_patch_sha256:[[:space:]]*//p' "${lock_file}"
)"
if [[ "${sdk_patch_sha256}" != "${lock_sdk_patch_sha256}" ]]; then
  echo "M20 SDK adapter patch checksum mismatch" >&2
  echo "expected=${lock_sdk_patch_sha256} actual=${sdk_patch_sha256}" >&2
  exit 7
fi

# Verify every patch identity before allowing any checkout to be modified.
git -C "${scan_target}" apply --check "${patch_file}"
# Upstream SCAN sources use CRLF while the integration edits are normalized to
# LF. Ignore only whitespace changes so the locked semantic context is still
# checked on both clean Linux and Windows-origin checkouts.
git -C "${scan_target}" apply --ignore-space-change --check "${integration_patch_file}"
git -C "${sdk_target}" apply --check "${sdk_patch_file}"
git -C "${scan_target}" apply "${patch_file}"
git -C "${scan_target}" apply --ignore-space-change "${integration_patch_file}"
git -C "${sdk_target}" apply "${sdk_patch_file}"

{
  echo "schema_version=1"
  echo "scan_planner_revision=${actual_scan_revision}"
  echo "deep_robotics_model_revision=${actual_model_revision}"
  echo "m20_sdk_deploy_revision=${actual_sdk_revision}"
  echo "scan_patch_sha256=${patch_sha256}"
  echo "scan_integration_patch_sha256=${integration_patch_sha256}"
  echo "m20_sdk_adapter_patch_sha256=${sdk_patch_sha256}"
} > "${target_workspace}/m20_workspace.lock"

echo "Prepared isolated workspace: ${target_workspace}"
echo "Project packages: ${#packages[@]}"
echo "SCAN-Planner: ${actual_scan_revision} + verified CPU/integration patches"
echo "Deep Robotics model: ${actual_model_revision}"
echo "M20 SDK: ${actual_sdk_revision} + verified cmd_vel adapter patch"
echo
echo "Next:"
echo "  source /opt/ros/humble/setup.bash"
echo "  cd ${target_workspace}"
echo "  rosdep install --from-paths src --ignore-src -r -y"
echo "  python3 -m pip install mujoco==3.10.0"
echo "  colcon build --symlink-install --packages-up-to m20_warehouse_inspection"
