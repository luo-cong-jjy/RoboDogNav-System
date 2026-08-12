# 2026-07-29 M20 SDK/Gazebo 运控适配测试

## 范围

本报告只覆盖阶段 6A-1：

- `m20_locomotion_control` 独立包构建；
- 运动意图分类与非放大速度约束；
- 安全话题到 SDK 话题的静态契约；
- SDK 默认不启动的 launch 防护；
- SDK 可选依赖清单与设计文档一致性。

本报告不声称 Gazebo RL 动力学、90°/180° 转向或真机行为已经通过。这些属于
6A-2/6A-3。

## 结果

隔离构建：

```text
colcon build --packages-select m20_locomotion_control
1 package finished
```

包级测试：

```text
colcon test --packages-select m20_locomotion_control
24 tests, 0 errors, 0 failures, 0 skipped
```

测试内容包括：

- 7 项运动意图/速度边界单元测试；
- 2 项 launch 与安全话题契约测试；
- copyright、flake8、CMake lint、pep257 和 XML lint；
- NaN/Inf 失效置零；
- 高曲率协调转弯限速；
- 横移时前进限速；
- 输出各分量不大于输入绝对值。

主项目相关静态回归：

```text
test_phase5_workspace.py + test_configuration.py
19 passed
```

launch 加载：

```text
ros2 launch m20_locomotion_control sdk_locomotion.launch.py --show-args
start_sdk default: false
```

SDK 可选依赖文件 YAML 解析通过，revision 为：

```text
ee289d475f2dedf0332b7542f2b173fa9e8d1456
```

## 放行结论

6A-1 通过。允许下一步实现 Gazebo 16 关节动力学桥。

在以下条件全部满足前，不允许把 `start_sdk` 改为默认 true：

- Gazebo 正确发布 `/JOINTS_DATA` 与 `/IMU_DATA`；
- `/JOINTS_CMD` 只有一个发布者和一个执行后端；
- 关节方向、零位、力矩上限和 IMU 坐标经过静态校验；
- 起立与 60 s 静止台架通过。
