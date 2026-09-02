# 地图目录

```text
factory/       仿真/Nav2 默认工厂栅格地图
pcd/raw/       原始 PCD，默认文件为 t100ipro_2026-07-14-11-56-57.pcd
pcd/grids/     PCD 转换生成的 PGM/YAML 和路径验证输出
```

这里后续放 Nav2 可直接加载的 2D 栅格地图：

```text
map.yaml
map.pgm
```

地图按链路分开管理：

- `factory_world_map.yaml/.pgm`：与 `factory_environment.world` 的静态几何同源的人工静态地图，
  供 MuJoCo 链路加载。MuJoCo 链路不启动 `slam_toolbox`，该地图仅由 `map_server`
  发布给 Nav2，同时作为代码雷达模拟器的环境输入。
- `m20_factory_slam.yaml/.pgm` 及带时间戳的地图：由 Gazebo/实机运行
  `slam_toolbox` 后保存，供保存地图导航（AMCL）使用。

因此两条链路不会共享建图过程：只有实时建图 launch 启动 `slam_toolbox`。

PCD 转换脚本默认读取 `pcd/raw/t100ipro_2026-07-14-11-56-57.pcd`，并将结果写入
`pcd/grids/`。未指定输出名时，文件名包含生成时间、裁剪高度/距离范围和分辨率。

保存 Gazebo/Nav2 在线 SLAM 地图：

```bash
cd /home/virdyn/robodog_nav_system
source install/setup.bash
MAP_NAME=factory_slam_$(date +%Y%m%d_%H%M%S)
ros2 run nav2_map_server map_saver_cli \
  -t /map \
  -f /home/virdyn/robodog_nav_system/src/m20_nav2_system/maps/${MAP_NAME} \
  --occ 0.65 \
  --free 0.25 \
  --fmt pgm \
  --mode trinary
```
