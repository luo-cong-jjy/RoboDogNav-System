# M20窄通道净空参数与基准场景开发记录

日期：2026-07-29

## 问题定义

原有“障碍物最小间隔0.70 m”实验只约束任意两个独立障碍物的边界间距，不保证
任务路径必须穿过一个给定宽度的通道，也没有把以下四层限制分开：

1. M20机身几何宽度；
2. SCAN双圆柱硬碰撞模型；
3. SCAN轨迹优化器的偏好净空；
4. 独立占据栅格碰撞保护及其离散化误差。

因此，原实验可用于比较障碍密度，不能回答“系统能正常穿过多窄的通道”。

## 参数审计

M20机身尺寸采用当前项目确认值：长0.82 m、宽0.51 m、高0.57 m。

原版SCAN的横向双圆柱半径为0.25 m，`optimization.dist0=0.20 m`。障碍点
先按0.25 m膨胀，轨迹优化器再从膨胀边界保留0.20 m偏好净空。因此原版直通道
的偏好宽度为：

```text
2 × (0.25 + 0.20) = 0.90 m
```

原生SCAN链路的独立碰撞保护使用0.25 m轮廓和0.05 m安全边界，连续几何硬下限
为0.60 m。但碰撞保护运行在占据栅格上，整格膨胀和机器人中心所在格的量化会
继续增加所需宽度，不能把0.60 m理解为可稳定通过宽度。

## 统一净空档位

新增四个显式ROS参数覆盖文件：

| 档位 | SCAN `dist0` | SCAN偏好直通道 | 可选栅格A*膨胀 | 用途 |
|---|---:|---:|---:|---|
| `vendor` | 0.20 m | 0.90 m | 0.60 m | 完整保留旧版参数 |
| `tight` | 0.10 m | 0.70 m | 0.35 m | 仅限窄道边界实验 |
| `balanced` | 0.15 m | 0.80 m | 0.40 m | 0.05 m地图上的验证候选 |
| `conservative` | 0.20 m | 0.90 m | 0.45 m | 正式0.10 m占据图默认 |

四档均不降低独立急停包络，仍使用0.25 m轮廓加0.05 m安全边界。

单层原版效果启动保持`vendor`默认。完整双场景RViz/MuJoCo系统改为
`conservative`默认，因为正式场景占据图分辨率为0.10 m；这也保留了原版SCAN
的0.90 m优化净空。

## 标准门洞实验

新增0.60/0.65/0.70/0.75/0.80/0.90 m六套可复现门洞：

- 每个场景只有上下两块静态墙体；
- 墙体延伸到场景边缘附近，不存在可供M20绕行的替代路线；
- 起点和终点位于门洞两侧，目标方向与门洞垂直；
- PCD和占据图分辨率统一为0.05 m；
- MuJoCo世界继续从同一JSON几何元数据生成，点云、保护和动力学墙体共用几何。

生成器为`tools/generate_narrow_passage_sweep.py`，启动入口为
`narrow_passage_mission_mujoco.launch.py`。

## 静态栅格结果

在0.05 m测试占据图、0.30 m独立保护半径下：

| 名义门洞 | 膨胀后连通性 |
|---:|---|
| 0.60 m | 封闭 |
| 0.65 m | 封闭 |
| 0.70 m | 封闭 |
| 0.75 m | 首个连通档位 |
| 0.80 m | 连通 |
| 0.90 m | 连通 |

这只证明静态中心路径是否存在。0.75 m仍低于`balanced`档的0.80 m优化偏好，
没有姿态/跟踪余量，必须标记为边界档，不能称为“正常通过”。

对正式0.10 m占据图，报告工具给出的保守栅格边界为“必须大于0.80 m”，因此
离散档位中的首个候选是0.90 m。当前正式系统的工程口径由此确定为：

- 0.70 m及以下：不允许作为可通行通道；
- 0.75/0.80 m：只在0.05 m地图和专用基准中验证；
- 0.90 m：正式0.10 m地图的首个正常候选；
- 实物场地最终验收还需加入定位、点云噪声和控制跟踪误差。

## 动态验证状态

本轮首次启动0.80 m无界面MuJoCo实验时，未加载项目CycloneDDS配置，部分节点
因参与者索引不足退出；该结果是通信环境错误，不是通道失败。按固定DDS配置重启
时，执行权限审批服务网络中断，动态实验未获执行许可。因此本记录没有把静态
连通结果写成MuJoCo通过结论，六档动态重复实验仍是后续验收项。

## 使用方法

查看参数级和栅格级门槛：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 run m20_scan_navigation m20_clearance_report \
  --profile conservative \
  --map-resolution 0.10
```

启动完整MuJoCo门洞实验：

```bash
ros2 launch m20_warehouse_inspection \
  narrow_passage_mission_mujoco.launch.py \
  width:=0.90 \
  clearance_profile:=conservative
```

第二终端启动单步穿越任务：

```bash
source /opt/ros/humble/setup.bash
source /home/virdyn/robodog_nav_system/install/setup.bash

ros2 run m20_warehouse_inspection m20_start_inspection \
  --mission-id narrow_passage_benchmark
```

边界实验可以把`width`替换为六个支持值，并把`clearance_profile`替换为
`tight/balanced/vendor/conservative`。每次只改变一个变量，结束上一轮后再启动
下一轮。
