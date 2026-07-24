# M20 轮足运动模式设计

本文记录 `m20_scan_planner` 后续从“原 SCAN-Planner 四足轨迹跟踪”演进到“M20 轮足模式智能切换”的设计思路。当前文档是方案草案，用于指导后续代码拆分、接口设计和实验验证。

## 背景

当前 `m20_scan_planner` 保留了原 SCAN-Planner 的局部规划主链路：

```text
局部地图 / 障碍物信息
        ↓
SCAN-Planner 生成 B 样条轨迹
        ↓
closed_loop_controller 跟踪轨迹并发布 cmd_vel
        ↓
m20_kinematic_sim 根据 cmd_vel 更新 body_pose
        ↓
m20_gait_publisher 发布 M20 JointState 可视化
```

这套链路已经把原版面向四足/Go2 的包适配到了 M20 包名、M20 机器人描述、M20 运动学仿真和 M20 关节可视化。但它目前仍然是一个理想化运动学模型，并没有真正表达 M20 的“轮式、腿式、轮腿混合”能力差异。

也就是说，当前系统解决的是：

```text
路径怎么规划
轨迹怎么跟踪
速度怎么发出来
```

但还没有解决：

```text
这段路应该用轮子跑，还是用腿走
什么时候轮式不合适
什么时候需要切到轮腿混合或腿式模式
规划层是否应该主动生成更适合轮式巡航的路径
```

## 核心判断

轮足机器人的优势不是简单地“把四足步态换成轮子”，而是让系统根据环境和轨迹选择运动形态：

- 在直线、缓弯、开阔、平坦区域，优先使用轮式运动。
- 在大曲率转弯、狭窄通道、贴近障碍物、需要精细调整的位置，切换到腿式或轮腿混合。
- 在台阶、门槛、复杂地形、非平面区域，切换到腿式跨越或轮腿协同。

因此，推荐增加一层 `m20_locomotion_selector`，作为规划层和底层执行层之间的运动模式决策层。

## 推荐架构

目标架构：

```text
scan_planner_node
  输出 planning/bspline
        ↓
m20_locomotion_selector
  分析轨迹、曲率、通道宽度、障碍距离、速度需求、地形信息
        ↓
输出：
  /cmd_vel
  /m20_locomotion_mode
  /planning/m20_execution_frozen
        ↓
M20 底层控制器或仿真节点
```

在这个架构里：

- `scan_planner_node` 继续专注于局部避障和轨迹生成。
- `m20_locomotion_selector` 决定当前轨迹片段适合什么运动模式。
- 底层 M20 控制器根据模式执行轮式、腿式或轮腿混合控制。

## 运动模式

建议先定义以下模式：

```text
WHEEL_CRUISE
  轮式巡航。适合直线或小曲率路径，速度可以较高。

WHEEL_SLOW_TURN
  轮式低速转弯。适合曲率中等但仍可由轮式底盘完成的路径。

HYBRID
  轮腿混合。适合需要一定机体姿态调整、局部避障余量较小、或曲率开始变大的区域。

LEGGED_TURN
  腿式转向。适合原地转向、小空间掉头、大角度转身。

LEGGED_STEP
  腿式跨越。适合台阶、门槛、坑洼、非平面地形。

STOP_OR_RECOVER
  停止或恢复。适合规划失败、障碍过近、定位异常、控制输出不可信等情况。
```

第一阶段可以先实现三个模式：

```text
WHEEL_CRUISE
HYBRID
LEGGED_TURN
```

这样能先验证“直线轮式跑、急转腿式转、过渡区混合”的核心闭环。

## 模式切换依据

### 轨迹曲率

从 B 样条或短时跟踪目标中估计：

```text
speed = sqrt(vx^2 + vy^2)
curvature = abs(yaw_rate) / max(speed, eps)
turn_radius = 1 / max(curvature, eps)
```

建议规则：

```text
curvature < k_wheel
  使用 WHEEL_CRUISE

k_wheel <= curvature < k_hybrid
  使用 WHEEL_SLOW_TURN 或 HYBRID

curvature >= k_hybrid
  使用 LEGGED_TURN
```

也可以直接用转弯半径判断：

```text
turn_radius > R_wheel_min
  轮式可行

turn_radius 接近 R_wheel_min
  轮腿混合

turn_radius < R_wheel_min
  腿式转向
```

### 航向误差

当前 `closed_loop_controller` 已经有 `heading_error_threshold`。当目标方向和机体朝向误差太大时，控制器会冻结轨迹时间并原地转向。

后续可以把这部分升级成模式选择：

```text
abs(yaw_error) 小
  轮式跟踪

abs(yaw_error) 中等
  低速轮式转向或 HYBRID

abs(yaw_error) 大
  LEGGED_TURN
```

### 通道宽度和障碍距离

轮式运动通常需要更稳定的通过空间，腿式/轮腿混合可以在狭窄处更细腻地调整。

可以从局部地图估计：

```text
clearance = 当前轨迹附近到障碍物的最小距离
```

建议规则：

```text
clearance > wheel_clearance
  允许 WHEEL_CRUISE

hybrid_clearance < clearance <= wheel_clearance
  使用 HYBRID，限速

clearance <= hybrid_clearance
  使用 LEGGED_TURN 或 STOP_OR_RECOVER
```

### 横向速度需求

当前 `closed_loop_controller` 会输出 `linear.y`。这对理想全向运动学模型没有问题，但对真实轮式底盘不一定成立。

建议：

```text
WHEEL_CRUISE
  尽量限制 linear.y 接近 0

HYBRID
  允许小范围 linear.y

LEGGED_TURN / LEGGED_STEP
  允许更强的横向或姿态调整能力
```

### 地形条件

如果后续有高度图、坡度、台阶检测或足端可达性判断，可以加入：

```text
ground_slope
step_height
roughness
wheel_contact_confidence
leg_step_required
```

在当前点云仿真阶段，可以先不做真实地形判定，只预留接口。

## 和规划层的关系

运动模式选择首先可以放在控制层/行为层实现，因为这样侵入小、验证快：

```text
规划器照常输出轨迹
控制层根据轨迹特征选择轮式或腿式
```

但长期来看，规划层也应该参与。原因是，如果规划器完全不知道轮式约束，它可能生成轮式不友好的轨迹：

- 很小半径急转
- 原地大角度掉头
- 大量横向速度 `linear.y`
- 贴障碍物擦边
- 频繁 S 型摆动
- 曲率变化过快

因此后续应逐步加入 mode-aware planning：

```text
轮式模式：
  惩罚大曲率
  惩罚大 yaw_rate
  惩罚大 lateral velocity
  偏好平滑、宽敞、低曲率路径

腿式/混合模式：
  允许更小转弯半径
  允许更复杂局部调整
  代价更高，速度更低

模式切换：
  加入切换代价，避免频繁 wheel/leg 抖动
```

## 第一阶段实现建议

第一阶段目标：不大改 SCAN-Planner 主算法，只在控制链路中增加模式选择。

新增节点：

```text
m20_locomotion_selector
```

输入：

```text
planning/bspline
body_pose
局部地图或障碍距离，可先选用 planning/data_display 或后续专门接口
```

输出：

```text
/cmd_vel
/m20_locomotion_mode
/planning/m20_execution_frozen
```

最小实现可以先替代当前 `closed_loop_controller`：

```text
scan_planner_node
        ↓ planning/bspline
m20_locomotion_selector
        ↓ /cmd_vel + /m20_locomotion_mode
m20_kinematic_sim 或真实 M20 底层
```

第一版模式规则：

```text
if abs(yaw_error) > yaw_legged_threshold:
  mode = LEGGED_TURN
  cmd_vel.linear = 0
  cmd_vel.angular.z = limited_yaw_rate

else if curvature < wheel_curvature_threshold and abs(linear_y_des) < lateral_threshold:
  mode = WHEEL_CRUISE
  cmd_vel.linear.x = forward_speed
  cmd_vel.linear.y = 0
  cmd_vel.angular.z = smooth_yaw_rate

else:
  mode = HYBRID
  cmd_vel.linear.x = reduced_forward_speed
  cmd_vel.linear.y = limited_lateral_speed
  cmd_vel.angular.z = limited_yaw_rate
```

为避免模式抖动，必须加入滞回：

```text
进入 HYBRID 的曲率阈值 > 退出 HYBRID 的曲率阈值
进入 LEGGED_TURN 的 yaw 阈值 > 退出 LEGGED_TURN 的 yaw 阈值
模式至少保持 min_mode_duration 秒
```

## 第二阶段实现建议

第二阶段目标：让规划更偏好轮式友好的路径。

可改内容：

- 在 B 样条优化中增加曲率或 yaw-rate 相关代价。
- 在轨迹评价中增加 `wheel_feasibility_score`。
- 对靠近障碍物的高速轮式路径增加代价。
- 对连续急转、S 弯、横向速度过大的轨迹增加代价。
- 在 RViz 中可视化当前模式、曲率、轮式可行性评分。

输出可以扩展：

```text
planning/m20_locomotion_hint
```

由规划器给控制器提示：

```text
preferred_mode
max_speed
max_yaw_rate
required_clearance
```

## 第三阶段实现建议

第三阶段目标：真正的 mode-aware planner。

规划时同时考虑：

```text
几何可行性
碰撞风险
轨迹平滑性
轮式可行性
腿式可行性
模式切换代价
能耗或时间代价
```

最终目标不是“能到”，而是：

```text
在开阔处轮式快速通过
在狭窄处腿式灵活调整
在复杂地形处轮腿协同
全程尽量减少不必要的模式切换
```

## 仿真验证指标

建议记录以下指标：

- 到达成功率
- 总耗时
- 平均速度
- 最大曲率
- 平均曲率
- 模式切换次数
- WHEEL_CRUISE 占比
- HYBRID 占比
- LEGGED_TURN 占比
- 距离障碍物最小 clearance
- 控制命令平滑性
- 是否出现频繁模式抖动

## 当前代码对应关系

当前相关文件：

- `src/closed_loop_controller.cpp`：现有 B 样条闭环跟踪器，后续可拆分或替换为 `m20_locomotion_selector`。
- `src/m20_kinematic_sim.cpp`：理想化 M20 运动学仿真，目前支持 `vx`、`vy`、`wz`。
- `src/m20_gait_publisher.cpp`：M20 JointState 可视化，目前根据里程计速度生成腿部摆动和轮子转动效果。
- `config/controllers.yaml`：控制器和仿真参数，后续可新增模式阈值参数。
- `launch/run.launch.py`：后续可新增 `controller_mode:=locomotion_selector` 或直接替换默认 closed-loop 控制器。

## 待办清单

- 定义 `m20_locomotion_mode` 消息或先用 `std_msgs/String`/`std_msgs/UInt8` 表达模式。
- 实现 `m20_locomotion_selector` 节点。
- 从 B 样条采样短时轨迹，计算局部曲率、期望速度、yaw error。
- 添加模式滞回和最小保持时间。
- 在 RViz 中显示当前模式。
- 在仿真里比较纯 closed-loop 和 mode selector 的效果。
- 再决定是否把模式偏好反向写入规划层代价函数。

