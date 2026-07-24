# M20 RoboSense Multicast Smoke Test

This is a minimal ROS2 workspace bundle for decoding the M20 RoboSense UDP multicast stream on an external host or GOS.

## What It Contains

```text
src/rslidar_msg
src/rslidar_sdk
config/rslidar_multicast_rshelios_backpack.yaml
config/rslidar_multicast_rshelios_gos.yaml
scripts/build.sh
scripts/run_backpack_rshelios.sh
scripts/run_gos_rshelios.sh
scripts/probe_points.sh
```

The current packet capture matches the `RSHELIOS` decoder better than `RSM1`:

```text
UDP payload length: 1248
MSOP id: 55 aa 05 5a
block id: ff ee
```

## Build On Backpack Host

```bash
cd ~/m20_rslidar_multicast_smoke
bash scripts/build.sh
```

## Run On Backpack Host

The backpack host wired robot interface observed earlier is `10.21.31.192`.

```bash
cd ~/m20_rslidar_multicast_smoke
bash scripts/run_backpack_rshelios.sh
```

Open another terminal:

```bash
cd ~/m20_rslidar_multicast_smoke
bash scripts/probe_points.sh
```

## Run On GOS 104

Only use this after copying/building the bundle on GOS.

```bash
cd ~/m20_rslidar_multicast_smoke
bash scripts/run_gos_rshelios.sh
```

## Expected Topics

```text
/m20/front/points
/m20/rear/points
```

If UDP packets are visible but no PointCloud2 is published, the first suspect is `lidar_type`. Try finding the official rslidar config on AOS/NOS/GOS and replace `RSHELIOS` with the real RoboSense model.
