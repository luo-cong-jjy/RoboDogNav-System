# 实物部署准备 04：M20-pao 逐步部署与单层导航验收手册

> 基线日期：2026-08-12  
> 开发机：Ubuntu 22.04 / ROS 2 Humble  
> 目标机：山猫 M20-pao 背部 x86，Ubuntu 20.04 / ROS 2 Foxy  
> 目标工作空间：`/home/m20/robodog_nav_system`  
> 当前最低验收目标：只完成 F1 单层手动导航，不执行自动巡检和楼层切换

本文是后续现场部署的操作主线。它从 Humble/MuJoCo 单层仿真验收开始，依次覆盖源码
冻结、实场配置、源码上传、Foxy 依赖安装、目标机原生构建、雷达与 LIO 验证、只读联调、
台架运动、低速空场运动、F1 单层导航、记录和回退。

本文不是“已经完成实机验收”的证明。当前代码已具备 Foxy 源码预检、Elevator-LIO
适配、`basic_server`/`direct_ros` 双运动后端和失效停车保护；目标机编译、真实地图、现场
坐标对齐、AOS 控制权及实机运动仍需按本文逐级放行。

## 1. 当前部署结论

### 1.1 第一轮推荐组合

首次实机部署固定使用下列组合：

| 项目 | 第一轮选择 |
| --- | --- |
| 主机 | 背部 x86，Ubuntu 20.04，ROS 2 Foxy |
| ROS 中间件 | `rmw_fastrtps_cpp` |
| 定位/实时点云 | Elevator-LIO |
| 规划 | 当前适配后的 SCAN-Planner |
| 安全与运动适配 | `m20_inspection_core` + `m20_locomotion_control` |
| 出厂运动最后一跳 | `basic_server` |
| direct ROS | 第一轮成功后再 A/B 验证 |
| 自动取得运动权 | 禁止，`auto_enable_motion=false` |
| 楼层范围 | 只激活、测试 F1 |
| 自动巡检 | 暂不启动 |
| 电梯/切层 | 暂不启动 |

`basic_server` 和 `direct_ros` 不能同时运行。两者都只接收统一安全速度
`/m20/locomotion/cmd_vel_sdk`，所以切换传输方式不应改 SCAN、任务或碰撞保护代码。

### 1.2 绝对禁止项

现场首次部署不得：

- 把 `dense_four_corner_system.yaml` 或其他仿真 PCD 当作实场地图；
- 复制开发机的 `build/`、`install/`、`log/` 到 Foxy 主机；
- 在未确认 AOS 控制权前设置 `command_ownership_confirmed=true`；
- 把 `auto_enable_motion` 默认改成 `true`；
- 同时运行机载原导航、自动充电、遥控速度程序和本项目运动后端；
- 为了“先跑起来”直接减小障碍膨胀、关闭碰撞保护或放宽定位超时；
- 在机器人落地、周围有人或没有物理急停人员时做首次 enable；
- 因 `basic_server` 连接失败而未经检查直接改用 `direct_ros`；
- 在只完成 F1 验收时启动包含 F1→F2 的现有自动任务。

### 1.3 单层硬件配置已经独立

`config/sites/m20_pao_f1_template.yaml` 是 fail-closed 的单层硬件模板。它显式声明
`deployment_mode: hardware` 和 `map_asset_kind: surveyed`，允许只配置 F1；真实 PCD、LIO
位姿和目标都必须在同一个 `world` 坐标系，`simulation_offset` 非零会被拒绝。

模板故意引用仓库中不存在的 `maps/sites/m20_pao/F1/static_map.*`，并且任务只停在起点。
完成建图、PCD 导入、现场坐标审核前，一键硬件启动会因缺少资产而安全失败，不能误用
仿真 F2 欺骗系统。多层仍需独立的楼层定位与运输 provider，F1 成功不等于多层完成。

## 2. 分阶段放行总表

任何阶段失败都停在本阶段，不允许跳到下一阶段。

| 阶段 | 内容 | 是否允许机器人运动 | 通过后才可进入 |
| --- | --- | --- | --- |
| G0 | Humble/MuJoCo F1 单层仿真验收 | 仿真运动 | 源码冻结 |
| G1 | 实场配置、LIO 配置、发布源码准备 | 否 | 上传 |
| G2 | Foxy 依赖、源码预检、原生构建、单元测试 | 否 | 传感器检查 |
| G3 | 双雷达、IMU、时间戳和网络检查 | 否 | LIO-only |
| G4 | Elevator-LIO 单独定位与建图/重定位检查 | 否 | 系统只读联调 |
| G5 | 完整系统启动，但没有运动权限 | 否 | 台架运动 |
| G6 | 支撑架上 enable、零速、急停和短脉冲 | 仅架空 | 空场低速 |
| G7 | 空旷区域低速直行、停车和转向 | 是，人工监护 | 单点导航 |
| G8 | F1 四个代表性目标并返回起点 | 是，人工监护 | 单层验收完成 |
| G9 | F1-only 自动任务 | 后续可选 | 多层开发 |
| G10 | 电梯、多层、systemd 生产化 | 当前不验收 | 最终生产 |

## 3. G0：开发机单层仿真验收与冻结

### 3.1 使用干净环境构建

在开发机执行：

```bash
cd /home/virdyn/robodog_nav_system
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to m20_warehouse_inspection \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source /home/virdyn/robodog_nav_system/install/setup.bash
```

建议在冻结前至少运行核心测试：

```bash
colcon test --packages-select \
  m20_warehouse_interfaces \
  m20_multifloor_map \
  m20_inspection_core \
  m20_scan_planner \
  m20_scan_navigation \
  m20_locomotion_control \
  m20_mujoco_backend \
  m20_warehouse_inspection \
  --return-code-on-test-failure

colcon test-result --verbose
```

任何测试失败都要先确认是代码错误还是测试环境缺失，不能只看“程序能启动”就冻结。

### 3.2 校验冻结仿真资产

```bash
SIM_SHARE="$(ros2 pkg prefix m20_warehouse_inspection)/share/m20_warehouse_inspection"

ros2 run m20_warehouse_inspection m20_validate_config \
  --config "$SIM_SHARE/config/dense_four_corner_system.yaml" \
  --package-root "$SIM_SHARE" \
  --require-assets
```

这里的 `--require-assets` 适用于项目生成的仿真地图。后续真实地图不能直接套用这个
确定性障碍物检查器。

### 3.3 启动 MuJoCo 完整链

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=79
unset ROS_LOCALHOST_ONLY
export CYCLONEDDS_URI=file:///home/virdyn/robodog_nav_system/install/\
m20_warehouse_inspection/share/m20_warehouse_inspection/config/cyclonedds_local.xml

ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py
```

等待日志出现：

```text
MuJoCo M20 backend ready: official SDK stand-up is stable
```

随后只在 F1 使用 RViz `2D Goal Pose`。本轮不运行 `m20_start_inspection`，不把目标放到
共享原点另一侧，也不触发切层。

### 3.4 当前最低仿真验收标准

F1 选择至少四个代表性目标：短直线、长直线、空旷 90° 转向、绕单障碍，最后返回本轮
起点。必须同时满足：

- 四个目标和返回目标均成功结束，不靠重新启动节点恢复；
- MuJoCo 无跌倒、非有限状态、执行器故障或持续接触；
- `/m20/sim/backend_fault` 为空；
- `/m20/control/collision_stop` 不长期锁存；
- SCAN 不出现持续的 `The robot is inside an obstacle` 重规划死循环；
- 机器人朝向和轨迹跟踪无持续发散；
- 停车后实测速度收敛，机器人不持续漂移；
- 全程 active floor 保持 F1，generation 不因误操作切换。

这只是当前约定的“仿真完整验收”最低标准。它不证明窄通道、多层、电梯和真机运动已经
通过。

### 3.5 冻结记录

验收后保存一份报告到：

```text
src/m20_warehouse_system/bringup/m20_warehouse_inspection/docs/test_reports/YYYY-MM-DD_f1_sim_acceptance.md
```

报告至少记录：Git commit 或源码 SHA、系统配置、五个目标、每个结果、异常日志和测试
人员。冻结前执行：

```bash
cd /home/virdyn/robodog_nav_system
git status --short
git diff --check
```

有意修改必须提交或在报告中逐项登记；不明来源的改动、临时参数和未解释的环境文件不能
进入实机版本。

## 4. G1：仿真验收后必须替换或新增的内容

迁移原则是“替换输入/输出后端，不改已经验收的规划与安全语义”。不要在同一轮同时改
真实地图、SCAN 代价、运动包络和任务点，否则现场出现问题时无法定位变量。

### 4.1 修改清单

| 内容 | 新文件/参数建议 | 操作要求 |
| --- | --- | --- |
| 实场系统配置 | `config/sites/m20_pao_warehouse.yaml` | 新建，不覆盖仿真 YAML |
| 每层静态地图 | `maps/sites/m20_pao/F1/`、`F2/` | PCD、JSON 一组一层；不生成 PGM/YAML |
| LIO 建图根配置 | `Elevator-LIO/yaml/root_config_m20_navigation.yaml` | 已提供；使用 mapping runtime |
| LIO 重定位根配置 | `Elevator-LIO/yaml/root_config_m20_navigation_relocation.yaml` | 已提供；加载稳定 F1 PCD 名 |
| 雷达/IMU 外参 | 新的 site sensor YAML | 用实机标定值，不猜测 |
| LIO 运行模式 | 新的 site runtime YAML | 明确 mapping 或 relocation |
| AOS 地址 | launch 参数 `robot_host` | 默认参考值 `10.21.31.103`，现场核对 |
| DDS | `RMW_IMPLEMENTATION`、`ROS_DOMAIN_ID` | 与 AOS/驱动一致，禁止残留 Cyclone URI |
| 运动传输 | `factory_transport` | 第一轮固定 `basic_server` |
| 任务 | site YAML 的 `mission.sequence` | 第一轮不启动；需要自动任务时只含 F1 |
| 楼层运输 | `floor_switch`/provider | 第一轮不调用，不能使用 `timed_hold` 实机运输 |

### 4.2 真实地图资产要求

当前地图服务器对每层要求：

```text
static_map.pcd       静态障碍物点云
static_map.json      来源、坐标、日期和校验信息
```

PCD 读取器当前只接受：

```text
FIELDS x y z intensity
DATA ascii
```

Elevator-LIO 保存的 PCD 可能是 binary 或包含不同字段，不能未经检查直接放入系统。真实
地图还必须满足：

- PCD 使用 LIO `world` 原点、x/y 朝向和米制单位；
- F1 的启动位姿、巡检点、障碍点和 LIO `world` 完全对齐；
- 删除动态人员、车辆和临时物体，保留墙、柱、货架等静态结构；
- `initial_pose` 和所有目标位于自由区，四周给 M20 双圆足迹留下余量；
- 现场先在 RViz 叠加静态 PCD、实时 `/LIO/clouds_lidar`、SCAN 在线占据点云和机器人
  模型检查；
- 不运行 `m20_generate_maps` 生成或覆盖真实资产。

仓库已经提供 `m20_import_site_pcd`：先用 PCL 把 Elevator-LIO binary PCD 转成 ASCII，
再由该工具只提取 `x/y/z/intensity`、删除非有限点并生成来源 SHA、范围和输出 SHA 的审计
JSON。硬件 profile 的 `--require-assets` 会校验这些字段，不会套用仿真随机障碍假设。
它不负责动态物体清理、地面/噪点过滤或坐标审核，这三项仍须现场完成。

### 4.3 单层验收 profile 的处理

第一轮直接复制单层硬件模板并只接入 F1。不要加入 F2，也不要运行 `floor_transfer`。
未来多层硬件 profile 可以让各层地图 x/y 重叠，但每次只能激活已经完成重定位的那一层；
这与仿真横向摆放的两块区域是不同模式。

### 4.4 LIO 模式和起点

`root_config_m20_navigation.yaml` 当前默认是 mapping：

```yaml
relocation_enable: false
```

它会以本次启动点建立 `world`。如果静态导航地图来自另一次建图，本次启动必须能恢复到
相同原点和朝向。可选方式是：

1. 每次在已标定的地图原点和朝向启动；或
2. 使用 Elevator-LIO relocation，加载对应 PCD，并按该项目要求在地图原点附近启动。

建图结束后使用 `root_config_m20_navigation_relocation.yaml`，它加载固定文件名
`src/Elevator-LIO/PCD/m20_pao_f1_scans.pcd`。当前 Elevator-LIO 没有“任意初始位姿重定位”
服务。没有验证重定位时，不允许在仓库任意
位置开机后直接导航。site LIO 配置还要按实机复核双雷达外参、IMU 外参、时间同步、blind
区、体素大小和帧名；基线中的数值只能作为已经验证硬件布局相同时的起点。

### 4.5 初期不应修改的参数

第一轮保持当前已验收的 SCAN 配置、双圆足迹和碰撞保护。实机专用能力配置使用：

```text
m20_locomotion_control/config/m20_factory_agile_flat_capabilities.yaml
```

该 profile 已明确 M20 机身约 `0.82 m × 0.506 m`，并使用保守速度上限。MuJoCo 中测得的
漂移补偿不能直接复制到出厂实机。实机直行、转向和停车数据出来以后，再单独建立
`m20_pao_factory_<date>.yaml` 做 A/B 测试。

## 5. G1：在开发机生成可上传的源码发布目录

不要把整个当前 `src/` 原样上传。当前工作空间包含旧实验包、第三方手册、调试数据和被
隔离包；应使用锁定脚本生成可复现闭包，再补入 Elevator-LIO 与 site 资产。

### 5.1 建立发布目录

以下示例中的日期标签必须换成当次真实发布号：

```bash
cd /home/virdyn/robodog_nav_system

RELEASE_TAG=20260811_r1
STAGE=/tmp/m20_pao_release_20260811_r1
mkdir -p "$STAGE"

src/m20_warehouse_system/bringup/m20_warehouse_inspection/tools/prepare_isolated_workspace.sh \
  "$STAGE" \
  --project-src /home/virdyn/robodog_nav_system/src \
  --scan-repository /home/virdyn/robodog_nav_system/src/third_party/SCAN-Planner \
  --model-repository /home/virdyn/robodog_nav_system/src/third_party/deep_robotics_model \
  --sdk-repository /home/virdyn/robodog_nav_system/src/third_party/sdk_deploy
```

`STAGE` 必须为空，脚本不会覆盖已有目录。它会固定 SCAN、官方模型和 SDK revision，验证
补丁 SHA，并放置必要的 `COLCON_IGNORE`。脚本末尾打印的 Humble 命令只是仿真指南；这里
只使用其“生成 source-only 闭包”功能，Foxy 编译必须在目标机进行。

### 5.2 补入 Elevator-LIO

```bash
rsync -a \
  --exclude '.git/' \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '*.bag' \
  /home/virdyn/robodog_nav_system/src/Elevator-LIO/ \
  "$STAGE/src/Elevator-LIO/"
```

如果目标机继续使用已经验证的外部 RoboSense 驱动，必须保证只有一个驱动所有者。保留
LIO 源码但隔离其内嵌 SDK：

```bash
touch "$STAGE/src/Elevator-LIO/rslidar_sdk/COLCON_IGNORE"
```

不要同时启动或编译两套向 `/rslidar_points_front`、`/rslidar_points_rear` 发布的驱动。

### 5.3 补入真实 site 配置和资产

如果真实资产是在发布目录生成后才完成，可再同步：

```bash
rsync -a \
  /home/virdyn/robodog_nav_system/src/m20_warehouse_system/bringup/m20_warehouse_inspection/config/sites/ \
  "$STAGE/src/m20_warehouse_system/bringup/m20_warehouse_inspection/config/sites/"

rsync -a \
  /home/virdyn/robodog_nav_system/src/m20_warehouse_system/bringup/m20_warehouse_inspection/maps/sites/ \
  "$STAGE/src/m20_warehouse_system/bringup/m20_warehouse_inspection/maps/sites/"
```

路径不存在说明 site 资产还没有完成，此时可以继续上传并构建 LIO，但不能执行 G5 的完整
导航启动。

### 5.4 生成源码清单

```bash
cd "$STAGE/src"
find . -type f \
  ! -path '*/__pycache__/*' \
  ! -path '*/.pytest_cache/*' \
  ! -name '*.pyc' \
  -print0 \
  | LC_ALL=C sort -z \
  | xargs -0 sha256sum \
  > "$STAGE/SOURCE_SHA256SUMS.txt"

source /opt/ros/humble/setup.bash
colcon list --base-paths "$STAGE/src" --topological-order \
  > "$STAGE/SOURCE_PACKAGES.txt"
```

人工检查 `SOURCE_PACKAGES.txt`：

- `drdds` 只能出现一次；
- `lio` 必须出现；
- SDK 中旧 `drdds` 不能再次出现；
- 旧工业巡检、Gazebo 沙箱和旧 Foxy 实验包不应出现。

### 5.5 对最终暂存源码再做一次 Humble 冒烟

准备脚本会从锁定 revision 重建第三方源码，不会带入第三方仓库中的未提交修改。因此真正
上传前，必须对 `$STAGE/src` 再构建一次，并至少重复 G0 的 F1 单层冒烟；不能只验收原
工作空间后就默认暂存目录完全等价。

```bash
cd "$STAGE"
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-up-to m20_warehouse_inspection \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

source "$STAGE/install/setup.bash"
ros2 launch m20_warehouse_inspection \
  inspection_mission_mujoco.launch.py
```

暂存源码的 F1 目标冒烟通过后，重新生成一次 `SOURCE_SHA256SUMS.txt`；之后不得再修改
`$STAGE/src`。`$STAGE/build`、`install` 和 `log` 只属于开发机验证，不上传目标机。

## 6. 上传到 M20-pao 同名工作空间

下列命令假设目标用户名是 `m20`、工作空间是 `/home/m20/robodog_nav_system`。把
`<M20_PAO_IP>` 换成实际背部主机地址；尖括号只是占位符，不能原样执行。

### 6.1 目标机先停止旧系统并留备份

先 SSH 登录，停止旧 launch/systemd 和任何项目速度发布者。确认机器人保持零速、物理
急停可用，然后建立 incoming 目录：

```bash
ssh m20@<M20_PAO_IP>

mkdir -p /home/m20/robodog_nav_system/src_incoming_20260811_r1
exit
```

如果 incoming 目录不是空目录，停止上传并换一个新 release tag，不能用 `rsync --delete`
覆盖来源不明的文件。

### 6.2 从开发机上传 source-only 内容

```bash
rsync -av --info=progress2 \
  "$STAGE/src/" \
  m20@<M20_PAO_IP>:/home/m20/robodog_nav_system/src_incoming_20260811_r1/

rsync -av \
  "$STAGE/SOURCE_SHA256SUMS.txt" \
  "$STAGE/SOURCE_PACKAGES.txt" \
  "$STAGE/m20_workspace.lock" \
  m20@<M20_PAO_IP>:/home/m20/robodog_nav_system/
```

不要上传 Humble 的 `build/`、`install/`、`log/`，也不要上传运行中的 rosbag 和临时 core
dump。

### 6.3 目标机校验并切换源码

```bash
ssh m20@<M20_PAO_IP>
cd /home/m20/robodog_nav_system/src_incoming_20260811_r1
sha256sum -c ../SOURCE_SHA256SUMS.txt
```

所有条目必须是 `OK`。之后才切换：

```bash
cd /home/m20/robodog_nav_system

# 仅在已有 src 时执行；备份目录必须事先不存在。
test ! -e src_backup_before_20260811_r1
mv src src_backup_before_20260811_r1
mv src_incoming_20260811_r1 src
```

如果目标工作空间原来没有 `src`，跳过第一条 `mv`。不要覆盖同名备份；源码回退依赖这个
目录。

## 7. G2：目标机环境、依赖与网络

### 7.1 核对操作系统

```bash
uname -m
lsb_release -a
source /opt/ros/foxy/setup.bash
printenv ROS_DISTRO
python3 --version
df -h /home/m20/robodog_nav_system
```

期望为 `x86_64`、Ubuntu 20.04、`foxy`、Python 3.8。任何一项不同都先停止，不要照抄
Foxy 构建参数。

### 7.2 安装基础工具与已确认 LIO 依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git rsync netcat-openbsd \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libeigen3-dev libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libtbb-dev \
  nlohmann-json3-dev libboost-all-dev libarmadillo-dev \
  libglm-dev libglfw3-dev libglew-dev \
  ros-foxy-pcl-ros ros-foxy-pcl-conversions \
  ros-foxy-rviz2 ros-foxy-rosbag2
```

初始化 rosdep。若目标镜像已经初始化，不要重复执行 `rosdep init`：

```bash
test -f /etc/ros/rosdep/sources.list.d/20-default.list \
  || sudo rosdep init

rosdep update --rosdistro foxy
```

随后按源码声明安装剩余依赖：

```bash
cd /home/m20/robodog_nav_system
source /opt/ros/foxy/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro foxy \
  -r -y
```

Foxy 已结束官方支持。若 `apt`/`rosdep` 因镜像源失效，不要用随机 pip 包替代 ROS 二进制；
先恢复机器狗厂商镜像中已验证的 Foxy apt 源并记录修改。实机不需要安装 MuJoCo Python
包，也不需要在开发机编译 Elevator-LIO。

### 7.3 固化专用运行环境

建议把目标机运行环境放在独立文件，而不是继续叠加开发机的 CycloneDDS `.bashrc`。创建：

```text
/home/m20/robodog_nav_system/deploy/m20_pao_foxy_env.sh
```

内容如下；`ROS_DOMAIN_ID=0` 只是官方指南示例，必须改成实机实际值：

```bash
source /opt/ros/foxy/setup.bash
source /home/m20/robodog_nav_system/install/20260811_r1/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
unset CYCLONEDDS_URI
```

首次调试先在终端手动 source。待整链验收后再决定是否从目标机 `.bashrc` 引用该文件。
禁止让 Humble、CycloneDDS 或开发机 domain 配置混入 Foxy 终端。

### 7.4 网络与控制源检查

```bash
ip -br address
ip route
ping -c 3 10.21.31.103
nc -vz 10.21.31.103 30001
```

`basic_server` 使用 TCP 30001 和 UDP 30000；TCP 检查成功不等于运动链已放行。现场还要
确认：

- 背部主机与 AOS 的实际 IP、子网和 MTU；
- AOS/basic_server 固件版本；
- 机载原 planner、自动充电和其他速度客户端已停止；
- 遥控器接管和物理急停操作人已经就位；
- 系统时间、雷达时间戳、IMU 时间戳没有跳变。

direct ROS 后续验证时还要检查：

```bash
ros2 topic info /NAV_CMD --verbose
ros2 topic info /MOTION_INFO --verbose
```

但第一轮 `basic_server` 不以发现这两个话题作为连接成功条件。

## 8. G2：Foxy 源码预检、构建与测试

### 8.1 检查包闭包和重复消息包

```bash
cd /home/m20/robodog_nav_system
source /opt/ros/foxy/setup.bash

colcon list --base-paths src --topological-order
colcon list --base-paths src | awk '$1 == "drdds" {print}'
```

第二条必须只输出一个 `drdds`，路径应指向当前工作空间的 canonical 包。然后运行两种后端
源码预检：

```bash
python3 src/m20_warehouse_system/bringup/m20_warehouse_inspection/tools/validate_foxy_hardware_source.py \
  --transport basic_server

python3 src/m20_warehouse_system/bringup/m20_warehouse_inspection/tools/validate_foxy_hardware_source.py \
  --transport direct_ros
```

两条都应为 PASS。输出中的 `MATCHES BACKPACK BASELINE` 表示活动 drdds 的 25 个消息字段
与最近部署在背部主机的 v1.2.0 源码一致；direct ROS 的 QoS 仍必须在目标 AOS 实测。

### 8.2 使用版本化目录原生构建

目标机不使用开发机的 install，也不在不同 release 之间复用 build。生产发布默认用复制
安装而不是 `--symlink-install`，便于以后回退到不可变 install。

先单独构建 Elevator-LIO：

```bash
cd /home/m20/robodog_nav_system
source /opt/ros/foxy/setup.bash
RELEASE_TAG=20260811_r1

colcon --log-base "log/$RELEASE_TAG/lio" build \
  --build-base "build/$RELEASE_TAG/lio" \
  --install-base "install/$RELEASE_TAG" \
  --packages-select lio \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DLIO_WITH_LIVOX=OFF \
    -DLIO_BUILD_SIM=OFF \
    -DLIO_BUILD_RVIZ_PLUGIN=OFF
```

加载 LIO install 后构建项目闭包：

```bash
source "/home/m20/robodog_nav_system/install/$RELEASE_TAG/setup.bash"

colcon --log-base "log/$RELEASE_TAG/project" build \
  --build-base "build/$RELEASE_TAG/project" \
  --install-base "install/$RELEASE_TAG" \
  --packages-up-to m20_warehouse_inspection \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

当前 `m20_warehouse_inspection` 的 package dependency 仍包含仿真闭包，所以这条命令会
构建部分实机不启动的包。这是当前最可复现的构建方式；不要在现场随意删包做“精简版”。
后续若要缩小生产体积，应通过正式拆分 hardware metapackage 和 package manifest 完成，
并重新做 Foxy 测试。

### 8.3 目标机单元测试

```bash
source "/home/m20/robodog_nav_system/install/$RELEASE_TAG/setup.bash"

colcon --log-base "log/$RELEASE_TAG/test" test \
  --build-base "build/$RELEASE_TAG/project" \
  --install-base "install/$RELEASE_TAG" \
  --packages-select \
    m20_warehouse_interfaces \
    m20_multifloor_map \
    m20_inspection_core \
    m20_scan_planner \
    m20_scan_navigation \
    m20_locomotion_control \
    m20_warehouse_inspection \
  --return-code-on-test-failure

colcon test-result \
  --test-result-base "build/$RELEASE_TAG/project" \
  --verbose
```

记录失败项。不能以“测试在 Humble 通过”为理由忽略 Foxy 失败。

### 8.4 接口和 launch 静态检查

```bash
ros2 pkg prefix drdds
ros2 interface show drdds/msg/NavCmd
ros2 interface show drdds/msg/MotionInfo
ros2 interface show drdds/msg/MotionState
ros2 interface show drdds/msg/Gait
ros2 interface show drdds/msg/StdMsgInt32

ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py --show-args
```

`ros2 pkg prefix drdds` 必须指向当前 release，不能指向厂商旧 overlay。上述输出还要与
`src/drdds-背部主机当前版` 留档对比；静态预检已核对源码 ABI，但目标机 install 仍须复核。

## 9. G3：双雷达和 IMU 单独验证

这一阶段不启动本项目运动后端。使用已经在 M20-pao 验证过的 RoboSense 驱动启动命令，
不要猜测新的 launch 文件名。启动后执行：

```bash
ros2 topic type /rslidar_points_front
ros2 topic type /rslidar_points_rear
ros2 topic type /IMU

timeout 15s ros2 topic hz /rslidar_points_front
timeout 15s ros2 topic hz /rslidar_points_rear
timeout 15s ros2 topic hz /IMU

ros2 topic echo /rslidar_points_front --once --field header
ros2 topic echo /rslidar_points_rear --once --field header
ros2 topic echo /IMU --once --field header
```

此前目标机记录中两路雷达约为 8.3～10 Hz，只能作为参考。现场通过标准是：

- 两路 PointCloud2 和 IMU 持续发布，无长时间断流；
- frame、时间戳单调递增，无未来时间和零时间；
- 双雷达没有两个驱动重复发布；
- 点云方向、量纲、近场遮挡和安装外参符合实物；
- CPU、内存和网卡没有持续丢包或饱和。

## 10. G4：Elevator-LIO 单独验证

### 10.1 启动 LIO-only

```bash
source /home/m20/robodog_nav_system/deploy/m20_pao_foxy_env.sh

ros2 launch lio start_ros2.launch.py \
  config_path:=root_config_m20_navigation.yaml \
  use_rviz:=false
```

该基线可用于接口检查和建图，但必须先确认其双雷达/IMU 外参与当前实物完全一致。实际
导航改用 `root_config_m20_navigation_relocation.yaml`，不能用 mapping 模式直接放行。

### 10.2 检查输出和 TF

```bash
ros2 topic type /LIO/clouds_lidar
ros2 topic type /LIO/odom_vehicle
ros2 topic type /LIO/odom_imu

timeout 15s ros2 topic hz /LIO/clouds_lidar
timeout 15s ros2 topic hz /LIO/odom_vehicle
timeout 15s ros2 topic hz /LIO/odom_imu

ros2 topic echo /LIO/odom_vehicle --once
ros2 run tf2_ros tf2_echo world base_link
```

必须确认：

- `/LIO/clouds_lidar` 已在 `world` 中，不被再次变换；
- `/LIO/odom_vehicle` 是 `world -> base_link`；
- TF 树中没有第二个节点同时发布 `world -> base_link`；
- 静止时无 NaN、时间倒退和明显跳变；
- 手推/遥控短距离移动时，坐标方向与 RViz/地图一致；
- 如果使用 relocation，冷启动重复三次都能回到同一地图坐标。

建议把 60 秒静止漂移、1 m 直线和一次 90° 转向记录进 rosbag，再根据实机噪声确定正式
阈值。首轮可用的保守排错门槛是：静止 60 秒位置漂移不超过 0.10 m、yaw 漂移不超过
2°、相邻帧无超过 0.20 m 的非物理跳变；超出时先检查标定和时间同步，不调 SCAN。

### 10.3 F1 建图：精确命令

先确保机器人停在以后可重复摆放的“地图原点”，记录地面标记、机身朝向和照片。启动
mapping profile：

```bash
source /home/m20/robodog_nav_system/deploy/m20_pao_foxy_env.sh

ros2 launch lio start_ros2.launch.py \
  config_path:=root_config_m20_navigation.yaml \
  use_rviz:=false 2>&1 | tee \
  /home/m20/robodog_nav_system/deployment_records/${RELEASE_TAG}/lio_mapping.log
```

用遥控器低速覆盖 F1；闭环走廊至少正反各一次，避免人员长期跟在雷达近场。完成后在这个
终端按一次 Ctrl-C，等待出现 `save pcd file:` 后再关驱动/断电。不能用 `kill -9`。
源码确认输出位于：

```bash
LIO_PCD_DIR=/home/m20/robodog_nav_system/src/Elevator-LIO/PCD
find "$LIO_PCD_DIR" -maxdepth 1 -type f -name '*_scans.pcd' \
  -printf '%TY-%Tm-%Td %TH:%TM:%TS %s %p\n' | sort
```

选择刚生成且非 `ikdtree` 的文件，保留原件并建立稳定导航名：

```bash
MAPPING_PCD=/home/m20/robodog_nav_system/src/Elevator-LIO/PCD/<时间戳>_scans.pcd
test -s "$MAPPING_PCD"
cp --preserve=timestamps "$MAPPING_PCD" \
  /home/m20/robodog_nav_system/src/Elevator-LIO/PCD/m20_pao_f1_scans.pcd
sha256sum "$MAPPING_PCD" \
  /home/m20/robodog_nav_system/src/Elevator-LIO/PCD/m20_pao_f1_scans.pcd
```

尖括号路径必须替换为上一条 `find` 的实际结果。随后停掉 mapping 进程，使用只加载地图、
不继续积累的 relocation profile 冷启动三次：

```bash
ros2 launch lio start_ros2.launch.py \
  config_path:=root_config_m20_navigation_relocation.yaml \
  use_rviz:=false
```

每次必须看到 `Load Points Numble` 大于零，机器人均从地面标记附近启动，并按 10.2 节检查
漂移和 `world -> base_link`。Elevator-LIO 的 relocation 仅支持原点附近启动；不满足时
不能用手工改初始位姿掩盖。

### 10.4 将 LIO 地图导入 SCAN 静态 PCD

Elevator-LIO 保存 binary PCD 且可能带 normal/curvature 字段。先转 ASCII 到临时文件：

```bash
mkdir -p /home/m20/robodog_nav_system/deployment_records/${RELEASE_TAG}/map_import

pcl_convert_pcd_ascii_binary \
  /home/m20/robodog_nav_system/src/Elevator-LIO/PCD/m20_pao_f1_scans.pcd \
  /home/m20/robodog_nav_system/deployment_records/${RELEASE_TAG}/map_import/f1_lio_ascii.pcd \
  0 8
```

复制模板为现场配置，填写真实地图范围、起点和审核点；不要覆盖模板：

```bash
SITE_SOURCE=/home/m20/robodog_nav_system/src/m20_warehouse_system/bringup/m20_warehouse_inspection
mkdir -p "$SITE_SOURCE/config/sites" "$SITE_SOURCE/maps/sites/m20_pao/F1"
cp "$SITE_SOURCE/config/sites/m20_pao_f1_template.yaml" \
  "$SITE_SOURCE/config/sites/m20_pao_warehouse.yaml"
```

编辑完成后执行受审计导入：

```bash
source /home/m20/robodog_nav_system/install/${RELEASE_TAG}/setup.bash

ros2 run m20_warehouse_inspection m20_import_site_pcd \
  --input /home/m20/robodog_nav_system/deployment_records/${RELEASE_TAG}/map_import/f1_lio_ascii.pcd \
  --output-pcd "$SITE_SOURCE/maps/sites/m20_pao/F1/static_map.pcd" \
  --output-metadata "$SITE_SOURCE/maps/sites/m20_pao/F1/static_map.json" \
  --floor F1 \
  --asset-frame world \
  --source-map-builder Elevator-LIO
```

工具默认拒绝覆盖已存在资产。若地图重建，使用新 release/目录；`--force` 只允许在已备份
且记录理由时使用。导入完成后重新构建 `m20_warehouse_inspection`，并把 PCD、JSON、site
YAML 和原 LIO PCD 同步回开发机作为版本真值。

不要一边 mapping 改写地图，一边把同一地图当作导航静态障碍真值。导入工具不清理动态
人员/车辆，也不验证墙体是否完整；清理和 RViz 人工叠加仍是必做验收。

## 11. G5：完整系统只读联调，仍禁止运动

### 11.1 校验 site YAML 和真实资产

```bash
SITE_ROOT="$(ros2 pkg prefix m20_warehouse_inspection)/share/m20_warehouse_inspection"
SITE_CONFIG="$SITE_ROOT/config/sites/m20_pao_warehouse.yaml"

ros2 run m20_warehouse_inspection m20_validate_config \
  --config "$SITE_CONFIG" \
  --package-root "$SITE_ROOT" \
  --require-assets
```

该命令校验 PCD 格式、point count、world frame、范围和 SHA；随后仍须人工检查静态 PCD
与实时 LIO 点云叠加正确。

### 11.2 启动完整系统但不确认控制权

如果 LIO 已由另一个终端运行：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  system_config:="$SITE_CONFIG" \
  start_elevator_lio:=false \
  use_rviz:=false \
  factory_transport:=basic_server \
  robot_host:=10.21.31.103 \
  command_ownership_confirmed:=false \
  auto_enable_motion:=false
```

如果由本入口启动 LIO，则删除 `start_elevator_lio:=false`，并补入 site
`lio_config_path:=root_config_m20_navigation_relocation.yaml`。

此时后端可以建立状态连接，但 `/m20/locomotion/backend_ready` 应为 false，运动 service
的 enable 应被拒绝。检查：

```bash
ros2 topic echo /m20/locomotion/backend_status --once
ros2 topic echo /m20/locomotion/backend_fault --once
ros2 topic echo /m20/localization/body_pose --once
ros2 topic echo /m20/map/state --once
ros2 topic echo /m20/control/safety_state --once
```

再执行一次故意的拒绝测试：

```bash
ros2 service call /m20/hardware/enable_motion \
  std_srvs/srv/SetBool "{data: true}"
```

结果必须明确包含“command ownership not confirmed”并保持零速。若机器人发生任何运动，
立即物理急停并停止项目，G5 判定失败。

### 11.3 RViz 只读检查

可在有显示器的目标机设 `use_rviz:=true`，或从同 domain 的 Foxy 调试机运行 RViz。必须
同时看到：

- M20 模型与 `base_link` 对齐；
- F1 静态地图与实时世界系点云重合；
- `/grid_map/occupancy_inflate` 与实体墙/货架位置合理；
- 点击 F1 目标后有 SCAN 路径/轨迹；
- 即使规划产生速度，最终硬件后端仍未 ready、机器人不动。

目标无响应时先检查 `/move_base_simple/goal`、地图 ready、LIO odom、点云和 planner
状态，不要先改规划参数。

## 12. G6：支撑架上首次取得运动权

### 12.1 安全前置条件

必须全部满足：

- 机器人由可靠支撑架托住，四轮离地且腿部有活动空间；
- 电池、急停、遥控器和现场保护人员就位；
- 机载原 planner、自动充电和其他 `/NAV_CMD`/basic_server 客户端已停止；
- `ros2 node list` 和厂商进程检查确认只有本项目最终运动后端；
- G5 的定位、地图、点云和安全状态均正常；
- 记录终端和 rosbag 已启动。

官方红色尾部旋转急停必须由一名独立保护员全程握持/可达。它切断关节电机动力，优先级
高于软件软急停，不能由本程序触发或释放。启动运动后端前先观察而不 enable：

```bash
ros2 topic echo /m20/locomotion/backend_status
```

`basic_server` 状态 JSON 必须包含 `hard_estop_known=true`、`hard_estop=false`。看不到 HES
状态或状态过期时后端按故障处理，不允许 enable。若使用 direct ROS，另检查：

```bash
ros2 topic type /HES_STATUS
timeout 5s ros2 topic echo /HES_STATUS --once
```

消息类型应为 `drdds/msg/StdMsgInt32`，未触发时 `value: 0`，触发时 `value: 1`。目标机
实际 QoS/字段不符时停止 direct ROS 放行，以背部主机已部署 drdds 和现场话题为准。

### 12.2 以“确认控制权但不自动 enable”重启

停止 G5 的完整系统，再执行：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  system_config:="$SITE_CONFIG" \
  start_elevator_lio:=false \
  use_rviz:=false \
  factory_transport:=basic_server \
  robot_host:=10.21.31.103 \
  command_ownership_confirmed:=true \
  auto_enable_motion:=false
```

`command_ownership_confirmed=true` 是操作员声明，不是自动检测。只有实际完成控制源清理后
才允许使用。

### 12.3 enable、状态机和零速检查

```bash
ros2 service call /m20/hardware/enable_motion \
  std_srvs/srv/SetBool "{data: true}"

ros2 topic echo /m20/locomotion/backend_status --once
ros2 topic echo /m20/locomotion/backend_ready --once
ros2 topic echo /m20/locomotion/measured_twist --once
```

`basic_server` 会推进控制使用模式、站立/RL 状态和 `0x3002` 步态。只有 status 显示
`ready=true`、状态正确、零命令时四轮不持续自转，才可做短脉冲。

### 12.4 短脉冲测试

先用最低有效前向速度做不超过 1 秒的架空脉冲，并立即补发零速：

```bash
timeout 1s ros2 topic pub -r 20 \
  /m20/control/cmd_vel_manual geometry_msgs/msg/Twist \
  "{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once \
  /m20/control/cmd_vel_manual geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

分别检查正向、反向和 yaw，单次只改变一个轴。侧向运动第一轮禁止。每次确认指令方向、
实测速度方向、轮腿动作和 300 ms 超时停车都正确。

### 12.5 急停与 disable

软件急停测试必须在支撑架阶段完成：

```bash
ros2 topic pub --once /m20/control/e_stop \
  std_msgs/msg/Bool "{data: true}"
```

确认立即零速和 AOS 软急停。解除软急停应按厂商规定操作，不能只发布 `false` 就假设状态
已经恢复。

随后只在支撑架上测试物理急停：保护员触发红色尾部旋钮，确认关节失去动力、backend
fault 为 `M20_HARD_ESTOP_ASSERTED` 且 enable 被拒绝。按官方手册人工释放后，机器人仍应
保持禁用；先检查姿态、支撑、电机和周边，再执行人工确认序列：

```bash
ros2 service call /m20/hardware/enable_motion \
  std_srvs/srv/SetBool "{data: false}"

ros2 topic echo /m20/locomotion/backend_status --once

ros2 service call /m20/hardware/enable_motion \
  std_srvs/srv/SetBool "{data: true}"
```

没有中间的 `false` 确认时，后端必须继续报告 `M20_HARD_ESTOP_RELEASE_LATCHED`。任何厂家
异步电机、驱动、电池、姿态或温度故障同样闭锁运动，而且不能由 enable/disable 清锁；
查明并消除硬件原因后，重启后端并从 G5 只读检查重新开始，不能脚本循环 enable。

普通测试结束使用：

```bash
ros2 service call /m20/hardware/enable_motion \
  std_srvs/srv/SetBool "{data: false}"
```

G6 通过标准：状态机正确、正负方向正确、命令超时停车、disable 停车、软件/物理急停均
有效，无异常腿姿、轮端持续空转或 backend fault。

## 13. G7：落地空场低速验证

选择平整、干燥、至少四周各留 2 m 的空场，设置人工边界和保护员。继续使用
`basic_server`、`auto_enable_motion=false`，每轮手工 enable。

按以下顺序一次只放行一个动作：

1. 0.15 m/s 前进 1 秒，停车；
2. 0.15 m/s 前进 3 秒，停车；
3. 低速后退 1 秒，停车；
4. 原地小角度 yaw，停车；
5. 90° 转向，记录实际扫掠空间；
6. 直线中加入小 yaw，确认轮腿能连续走曲线；
7. 软件 disable；
8. 遥控/物理急停接管。

记录命令速度、`measured_twist`、LIO 位姿、停车距离、最大横向漂移、yaw 误差和 M20
占用空间。出现明显侧漂、方向符号相反、停车超出安全距离或 LIO 跳变时，不能进入自动
导航。

## 14. G8：F1 单层手动导航验收

### 14.1 启动和放行

保持真实 site 配置、LIO 和完整硬件入口运行。先确认：

如果本轮重启过硬件入口，应先重新核对控制权和现场安全，再手动调用 enable；服务成功后
等待 backend ready，不能因为上一轮已经 enable 就假设状态延续。

```bash
ros2 topic echo /m20/map/active_floor --once
ros2 topic echo /m20/map/ready --once
ros2 topic echo /m20/locomotion/backend_ready --once
ros2 topic echo /m20/control/safety_state --once
```

active floor 必须是 F1，map ready 和 backend ready 必须为 true，安全状态无 hold。第一轮
导航不运行 `m20_start_inspection`。

### 14.2 目标序列

在 F1 选取并记录五个目标：

1. 起点前方约 1 m 的同向目标；
2. 空旷区 3～5 m 直线目标；
3. 空旷区需要约 90° 转向的目标；
4. 有单个静态障碍、但通道明显宽于机身和安全裕量的绕障目标；
5. 返回本轮起点。

开始时避免窄通道、玻璃/低矮悬空障碍和与当前朝向完全相反的狭窄目标。每个目标单独使用
RViz `2D Goal Pose` 下发，成功并完全停车后再发下一个。

### 14.3 同步记录 rosbag

```bash
mkdir -p /home/m20/robodog_nav_system/deployment_records/20260811_r1

ros2 bag record \
  -o /home/m20/robodog_nav_system/deployment_records/20260811_r1/f1_navigation \
  /m20/localization/body_pose \
  /LIO/odom_vehicle \
  /LIO/odom_imu \
  /LIO/clouds_lidar \
  /m20/locomotion/measured_twist \
  /m20/locomotion/cmd_vel_sdk \
  /m20/locomotion/backend_status \
  /m20/locomotion/backend_fault \
  /m20/control/cmd_vel_safe \
  /m20/control/safety_state \
  /m20/control/collision_stop \
  /m20/control/collision_guard_diagnostic \
  /m20/navigation/cmd_vel_raw \
  /planning/bspline \
  /grid_map/occupancy_inflate \
  /m20/map/state \
  /tf /tf_static
```

点云数据量大，首次记录建议每组目标后停止并归档，避免磁盘占满。direct ROS A/B 时再加
`/MOTION_INFO` 和 `/NAV_CMD`。

### 14.4 当前单层验收通过标准

五个目标必须全部满足：

- 导航返回成功，且无需重启 planner、地图或运动后端；
- 机器人实体未触碰障碍、围栏和人员；
- 跟踪方向合理，没有持续切向、横摆或轨迹两侧来回振荡；
- 停车位置、yaw 误差不超过 site profile 设定容差；
- `/m20/locomotion/backend_fault` 为空；
- LIO 无失锁、NaN、时间跳变或 `world -> base_link` 冲突；
- 没有持续 `The robot is inside an obstacle`、A-star 死循环或永久 collision hold；
- 未激活 F2，未发生 generation 切换；
- disable 和物理急停在最后复验有效；
- rosbag、launch 日志、配置 SHA 和操作记录完整。

任一目标失败都不能通过。先根据 rosbag 区分地图错位、定位漂移、规划贴边、运动跟踪误差
或厂家后端状态，不以减小膨胀半径作为通用修复。

## 15. G9：可选的 F1-only 自动巡检

当前最低验收不要求自动任务。需要进入该阶段时，应在 site YAML 中建立新的 mission ID，
sequence 只包含 F1 inspection 点和返回 F1 `initial_pose` 的 terminal；配置仍可保留 F2
资产，但任务不能包含 `floor_transfer`。

示意结构：

```yaml
mission:
  id: m20_pao_f1_patrol
  loop: false
  sequence:
    - {type: inspection, floor: F1, point: F1_LOWER_LEFT}
    - {type: inspection, floor: F1, point: F1_LOWER_RIGHT}
    - {type: inspection, floor: F1, point: F1_UPPER_RIGHT}
    - {type: inspection, floor: F1, point: F1_UPPER_LEFT}
    - {type: terminal, floor: F1, location: initial_pose, name: F1_RETURN}
```

所有点必须换成实场审核坐标。启动命令：

```bash
ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id m20_pao_f1_patrol
```

自动巡检至少连续成功三轮且每轮回到起点，才可视为 G9 通过。这仍不放行电梯和多层。

## 16. direct ROS A/B 验证

只有 `basic_server` 完成 G6～G8 后才测试 direct ROS。先停止 basic 后端，确认 M20 固件
满足开发指南要求、Fast DDS 和 domain 一致，再用：

```bash
ros2 launch m20_warehouse_inspection \
  inspection_mission_hardware.launch.py \
  system_config:="$SITE_CONFIG" \
  start_elevator_lio:=false \
  use_rviz:=false \
  factory_transport:=direct_ros \
  command_ownership_confirmed:=true \
  auto_enable_motion:=false
```

检查 `/MOTION_INFO` 约 20 Hz、QoS、`/MOTION_STATE`、`/GAIT`、`/NAV_CMD` 以及厂家 500 ms
看门狗。按 G6→G7→G8 从头重走，不允许因为 basic 已通过就跳过台架。

## 17. 故障停止和回退

### 17.1 任意异常的立即动作

优先级如下：

1. 物理急停/遥控接管；
2. 确认人员安全；
3. 调用 `/m20/hardware/enable_motion` false；
4. 停止 launch；
5. 保存 rosbag、终端日志和现场照片；
6. 禁止原地反复重试同一危险动作。

### 17.2 源码回退

由于源码和 install 都使用 release tag，回退时停止所有进程，恢复旧 `src_backup_*`，并
source 对应旧 install。不要执行 `git reset --hard` 或删除当前失败 release；保留它用于
故障复盘。

生产构建未使用 `--symlink-install`，所以旧 `install/<release>/` 不会因为当前 `src` 内容
变化而静默改变。确认旧版本后再重新启动，依然保持
`command_ownership_confirmed=false`、`auto_enable_motion=false` 做一次 G5。

## 18. 常见故障定位顺序

### 18.1 `ros2 topic list` 无输出

依次检查：

```bash
printenv RMW_IMPLEMENTATION
printenv ROS_DOMAIN_ID
printenv ROS_LOCALHOST_ONLY
printenv CYCLONEDDS_URI
ros2 daemon stop
timeout 10s ros2 topic list --no-daemon --spin-time 5
```

Foxy 实机应使用 Fast DDS，domain 与实际系统一致，且不应带开发机 Cyclone URI。

### 18.2 RViz 点击目标无反应

依次检查：

- `/move_base_simple/goal` 是否收到目标；
- `/m20/map/ready`、active floor 和 generation；
- `/m20/localization/body_pose` 是否持续；
- `/LIO/clouds_lidar` 与地图是否同 frame、同原点；
- SCAN 节点、`/planning/bspline` 和导航状态；
- safety state、collision stop 和 backend ready。

不要先改 `.bashrc`、goal topic 或膨胀参数。

### 18.3 `The robot is inside an obstacle`

先叠加检查实体位置、LIO 位姿、静态地图、膨胀图和双圆足迹。如果实体尚未碰撞但规划起点
落入栅格，常见原因是地图/LIO 错位、跟踪误差、时间延迟或轨迹贴边。先用 rosbag量化，
不能只凭终端文字认定实体已经碰撞，也不能直接把膨胀半径减小。

### 18.4 后端不能 ready

查看 `/m20/locomotion/backend_status` JSON：

- `connected=false`：网络、端口、basic_server 服务；
- `command_ownership_confirmed=false`：按设计拒绝；
- `motion_requested=false`：尚未调用 enable；
- 状态/步态不符：检查 AOS 状态机和是否有其他控制源；
- status timeout：网络、固件或反馈链；
- soft e-stop latched：按厂家步骤解除，不循环 enable。
- `HARD_ESTOP_STATUS_UNAVAILABLE/TIMEOUT`：HES 未收到或已过期，禁止运动；检查
  BasicStatus 或 `/HES_STATUS`，不能绕过。
- `M20_HARD_ESTOP_ASSERTED`：物理急停仍触发；人工排险并释放。
- `M20_HARD_ESTOP_RELEASE_LATCHED`：已释放但尚未人工确认；检查后调用 enable false，
  再由操作员重新 true。
- `M20_DEVICE_ERROR:*`：厂家异步设备故障；保存完整 status/日志，按官方错误码处理；
  enable/disable 不清锁，排障后重启后端并从 G5 复验。

## 19. 现场归档目录

每次 release 建议保存：

```text
/home/m20/robodog_nav_system/deployment_records/<release>/
├── SOURCE_SHA256SUMS.txt
├── SOURCE_PACKAGES.txt
├── m20_workspace.lock
├── environment.txt
├── network.txt
├── interfaces.txt
├── sensor_rates.txt
├── tf_tree.txt
├── lio_static_and_motion/
├── f1_navigation/
├── launch.log
└── acceptance.md
```

`acceptance.md` 至少由部署人员、急停人员、日期、机器人序列号、AOS/固件版本、地图版本、
transport、五个目标结果和未解决问题组成。没有记录的“现场成功一次”不作为发布证据。

## 20. 后续 systemd 生产化

G8/G9 通过前不要配置开机自动运动。通过后可建立只启动系统、仍不自动 enable 的
systemd service：

```ini
[Unit]
Description=M20-pao warehouse inspection
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=m20
WorkingDirectory=/home/m20/robodog_nav_system
ExecStart=/bin/bash -lc 'source /home/m20/robodog_nav_system/deploy/m20_pao_foxy_env.sh; exec ros2 launch m20_warehouse_inspection inspection_mission_hardware.launch.py system_config:=/home/m20/robodog_nav_system/install/20260811_r1/m20_warehouse_inspection/share/m20_warehouse_inspection/config/sites/m20_pao_warehouse.yaml use_rviz:=false factory_transport:=basic_server command_ownership_confirmed:=false auto_enable_motion:=false'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

这个 commissioning service 只读启动，enable 会被拒绝。未来若生产服务需要把
`command_ownership_confirmed` 设为 true，必须先有独立的控制权互锁、急停监控、运行区
清场和开机自检；`auto_enable_motion` 仍建议保持 false，由上位 supervisor 显式放行。

## 21. 多层阶段仍需完成的工作

F1 通过后，多层部署还缺少：

1. 硬件叠层地图 schema，去掉平面区域不重叠假设；
2. 每层真实静态地图及同一语义坐标约定；
3. 可重复的重定位/地图切换 provider；
4. 楼层身份、门状态、呼梯和选层接口；
5. 驶入/驶出电梯时独占而受安全仲裁的运动阶段；
6. `external_action + wait_for_target` 实机楼层事务；
7. Elevator-LIO 电梯段与到层后的定位质量门控；
8. 多层故障回退与人工接管测试。

当前 `timed_hold + preserve` 是平面仿真近似，不能直接部署成真实电梯动作。

## 22. 现场开始前还需补齐的信息

实际部署前请把下列信息写入当次 acceptance 记录：

- M20-pao 背部主机真实 IP、用户名和网卡；
- AOS/basic_server IP、端口、固件版本；
- `ROS_DOMAIN_ID` 和 direct ROS 实际 QoS；
- 已验证 RoboSense 驱动的包名、launch 命令和版本；
- 前后雷达、IMU 的准确外参与时间同步方式；
- F1/F2 地图原点、启动位姿、地图版本和资产 SHA；
- 机载 planner、自动充电和遥控控制权的停止/接管流程；
- 物理急停方式、软件急停恢复步骤和现场负责人；
- 是否只做 mapping，还是已经具备可重复 relocation。

缺少真实地图/坐标对齐会阻塞 G5；缺少控制权和急停流程会阻塞 G6；缺少 direct ROS QoS
只阻塞 direct 后端，不阻塞第一轮 `basic_server`。
