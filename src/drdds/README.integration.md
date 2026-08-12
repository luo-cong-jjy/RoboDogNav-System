# `drdds` 集成状态

本目录是项目唯一启用的 M20 DrDDS 接口包。`src/drdds-背部主机当前版`
是从 M20-PRO 背部主机取得的已部署基准，由 `COLCON_IGNORE` 隔离，只用于对照。
活动包的 wire ABI 以该基准为唯一真值：

- `MetaType` 使用 `builtin_interfaces/Time stamp`；
- `MotionInfoValue.state` 是 `int32`，`gait` 是 `uint32`；
- 不包含独立的 `Timestamp.msg`。

开发指南中部分截图展示过旧的 `Timestamp timestamp` 和嵌套
`MotionStateValue/GaitValue`，不能用它覆盖目标机已部署接口。静态预检会按背部
主机基准逐字段校验高层运动消息。

`src/third_party/sdk_deploy/src/drdds` 是厂商 SDK 的旧同名副本，同样由
`COLCON_IGNORE` 隔离。部署前还必须用 `ros2 interface show` 和
`ros2 topic info --verbose` 核对目标 AOS 当前类型与 QoS。

实机入口支持 `factory_transport:=basic_server` 和 `factory_transport:=direct_ros`；默认仍为
厂家优先建议且内置状态推进的 `basic_server`，导航、安全和巡检上层不依赖该选择。
