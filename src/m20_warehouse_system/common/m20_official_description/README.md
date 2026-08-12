# M20 官方模型 ROS 2 包装

本包包装云深处官方 `deep_robotics_model` 仓库的 M20 低分辨率 URDF 模型。

官方来源：

```text
repository: https://github.com/DeepRoboticsLab/deep_robotics_model.git
revision: 6113c62da96295e8d53abbc079af5296bf4649f8
license: BSD-3-Clause
```

资产处理规则：

- `urdf/vendor/M20.urdf` 是官方文件的逐字节副本。
- `meshes/*.STL` 是官方 M20 URDF mesh 的逐字节副本。
- `urdf/m20_official.urdf` 只移除非 URDF 的 MuJoCo compiler 块，并把 mesh URI 改成
  ROS 2 可解析的 `package://` URI。
- `xacro/m20.urdf.xacro` 仅作为兼容入口包含上述 URDF，不增加雷达、IMU、关节或几何。
- RViz 系统直接加载 `urdf/m20_official.urdf`，可见模型中没有项目自定义传感器。
- 仿真点云的传感器位姿属于消息级仿真接口，不属于官方可视模型。

独立查看：

```bash
ros2 launch m20_official_description display.launch.py
```
