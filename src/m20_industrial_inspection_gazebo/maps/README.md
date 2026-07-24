# 地图目录

这里后续放 Nav2 可直接加载的 2D 栅格地图：

```text
map.yaml
map.pgm
```

当前阶段先使用 SLAM Toolbox 在线建图，不强制提供静态地图。

官方 lightning 生成的点云地图位于：

```text
/home/virdyn/robodog_nav_system/data/office4f/global.pcd
```

后续如果要从点云转换为 2D 栅格地图，应把转换结果放在本目录，避免和官方数据混在一起。

保存 Gazebo/Nav2 在线 SLAM 地图：

```bash
cd /home/virdyn/robodog_nav_system
source install/setup.bash
MAP_NAME=factory_slam_$(date +%Y%m%d_%H%M%S)
ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f /home/virdyn/robodog_nav_system/src/m20_industrial_inspection_gazebo/maps/${MAP_NAME} \
  --occ 0.65 \
  --free 0.25 \
  --fmt pgm \
  --mode trinary
```
