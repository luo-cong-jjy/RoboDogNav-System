# M20 Lightning-LM 真机部署手册

本文档记录在山猫 M20 上复现、分析并部署 `lightning-lm-deep-robotics`、雷达建图/定位与后续导航代码的当前流程。最初目标是让外接背部主机直接看到原始 RoboSense 点云；结合硬件架构和实测结果后，当前判断已经调整为：

1. 原始 `/LIDAR/POINTS` 默认主要在 AOS 103 的 10.21.33 雷达网段可用。
2. GOS 104 和外接背部主机默认在 10.21.31 业务网段，当前看不到原始 `/LIDAR/POINTS`。
3. 背部主机能看到部分官方处理后的低频可通行区域话题，但频率约 1.43Hz，不适合作为高速动态避障主输入。
4. 后续代码部署需要在 AOS/GOS/背部主机之间做明确分工：高频感知靠 AOS 或桥接，导航主算法优先放在资源合适且能拿到高频点云的主机上。
5. 最新资源审计显示，GOS 104 当前负载最低，最适合作为后续导航主算法候选；NOS 106 已运行官方 passable/localization/localPlanner 等链路，负载最高，不应再叠加重计算算法。

当前仓库中的 Lightning-LM 路径为：

```bash
~/robodog_nav_system/src/third_party/lightning-lm-deep-robotics
```

所以下面的命令都按这个路径写。第三方 README 中部分命令使用 `src/lightning-lm-deep-robotics/...`，在本工作区需要改成 `src/third_party/lightning-lm-deep-robotics/...`。

## 1. 先解决背部主机看不到 `/LIDAR/POINTS`

这个问题通常不在 Lightning-LM，而在 M20 官方 ROS 环境、DDS 网络、NOS/AOS 组播转发或话题名不一致。

### 1.1 在外接背部主机上确认 ROS 环境

外接背部主机没有 `/opt/robot/scripts/setup_ros2.sh` 是正常的。这个脚本属于机器狗内部 AOS/NOS/GOS 官方系统，不属于外接主机。

外接背部主机每个新终端应 source 自己安装的 ROS2。Ubuntu 20.04 通常是 Foxy：

```bash
source /opt/ros/foxy/setup.bash
```

如果背部主机装的是 Humble，则改为：

```bash
source /opt/ros/humble/setup.bash
```

然后检查：

```bash
echo $ROS_DISTRO
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
echo $RMW_IMPLEMENTATION
```

真机多主机通信时，通常要求：

```bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

如果官方系统使用了固定 `ROS_DOMAIN_ID`，背部主机和发布雷达点云的主机必须一致。不要在没有确认官方配置前随手改 Domain ID；先记录当前值。

建议在外接背部主机上创建一个固定环境脚本：

```bash
cat > ~/m20_ros_env.sh <<'EOF'
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
EOF
```

之后每个终端执行：

```bash
source ~/m20_ros_env.sh
```

如果要强制 FastDDS 使用背部主机连接机器狗的网卡 `10.21.31.192`，可继续配置 FastDDS profile：

```bash
mkdir -p ~/.ros
cat > ~/.ros/m20_fastdds.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>m20_udp_transport</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>10.21.31.192</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>

  <participant profile_name="m20_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>m20_udp_transport</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
```

然后把下面一行追加到 `~/m20_ros_env.sh`：

```bash
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/.ros/m20_fastdds.xml' >> ~/m20_ros_env.sh
```

重新开终端后确认：

```bash
source ~/m20_ros_env.sh
echo $FASTRTPS_DEFAULT_PROFILES_FILE
```

### 1.2 在外接背部主机上搜索真实点云话题名

先不要只盯着 `/LIDAR/POINTS`，确认是否换了名字：

```bash
ros2 topic list | sort
ros2 topic list | grep -Ei "lidar|point|points|rslidar|robosense"
```

如果能看到类似 `/rslidar_points`、`/rslidar_points_0`、`/lidar_points`，但看不到 `/LIDAR/POINTS`，说明是话题名不一致。后续可以把 Lightning-LM 配置里的：

```yaml
common:
  lidar_topic: "/LIDAR/POINTS"
```

改成实际话题名。

如果完全看不到任何点云话题，继续往下查网络和组播。

### 1.3 在 NOS 上启动组播转发

M20 官方说明里，点云跨主机可见通常依赖 NOS 侧的 `multicast-relay.service`。在 AOS 上登录 NOS：

```bash
ssh user@10.21.31.106
```

在 NOS 上执行：

```bash
sudo systemctl start multicast-relay.service
sudo systemctl status multicast-relay.service
```

如果确认服务正常，可以设置开机自启：

```bash
sudo systemctl enable multicast-relay.service
```

回到外接背部主机后重新开一个终端：

```bash
source ~/m20_ros_env.sh
ros2 topic list | grep -Ei "lidar|point|points|rslidar|robosense"
ros2 topic hz /LIDAR/POINTS
```

### 1.4 当前实测拓扑下的重点判断

当前实测信息：

```text
背部主机远程 IP:        192.168.112.146
背部主机连接机器狗 IP:   10.21.31.192
AOS 103 机器狗侧 IP:     10.21.31.103, 10.21.33.103 等
NOS 106 机器狗侧 IP:     10.21.31.106, 10.21.33.106 等
前/后激光雷达网段:       10.21.33.201, 10.21.33.202
```

在 AOS 103 上执行官方环境脚本时，日志显示：

```text
Detected eth0 IP: 10.21.33.103
Selected IP(s) based on eth0 IP (10.21.33.103): 10.21.33.103
```

这说明 AOS 上的 FastDDS/DrDDS 配置优先选择了 `10.21.33.103` 这张雷达网段网卡。背部主机接入的是 `10.21.31.192`，如果 10.21.33 与 10.21.31 之间的 DDS 组播或路由没有打通，就会出现：

```text
AOS 103 上能看到 /LIDAR/POINTS
背部主机上能看到部分基础话题
背部主机上看不到 /LIDAR/POINTS 或收不到数据
```

此时优先检查三件事。

在背部主机上：

```bash
source ~/m20_ros_env.sh
unset ROS_LOCALHOST_ONLY

ip -br addr
ip route
ping -c 3 10.21.31.103
ping -c 3 10.21.31.106
ping -c 3 10.21.33.103
ros2 topic list | grep -Ei "lidar|point|points|rslidar|robosense"
```

判断：

```text
能 ping 10.21.31.103，但不能 ping 10.21.33.103:
  说明背部主机只通 10.21.31 网段，AOS 雷达网段不可达。

能看到 /LIDAR/POINTS，但 ros2 topic hz 收不到:
  优先按 QoS 问题查，见下一节。

完全看不到 /LIDAR/POINTS:
  优先查 NOS multicast-relay、DDS 网卡选择、ROS_DOMAIN_ID。
```

在 NOS 上确认组播转发：

```bash
ssh user@10.21.31.106
sudo systemctl restart multicast-relay.service
sudo systemctl status multicast-relay.service
ip -br addr
```

在 AOS 上确认 `/LIDAR/POINTS` 的发布者确实存在：

```bash
ssh user@10.21.31.103
su
source /opt/robot/scripts/setup_ros2.sh
ros2 topic info -v /LIDAR/POINTS
ros2 topic hz /LIDAR/POINTS
```

如果 `ros2 topic info -v /LIDAR/POINTS` 显示发布者在 AOS 上，而背部主机完全发现不到这个话题，下一步应重点处理 DDS 跨网段发现，而不是修改 Lightning-LM。

### 1.4.1 本轮外接背部主机实测结论

本轮在外接背部主机 `M20piper` 上执行了外接主机环境配置：

```bash
source /opt/ros/foxy/setup.bash
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

注意曾误输入过：

```bash
unset ROS_LOCALHOST_ID=0
```

这是无效命令。正确变量名是：

```bash
unset ROS_LOCALHOST_ONLY
```

随后创建 `~/m20_ros_env.sh`，并通过 FastDDS profile 强制外接主机只使用机器狗网线口：

```xml
<interfaceWhiteList>
  <address>10.21.31.192</address>
</interfaceWhiteList>
```

外接背部主机网卡状态：

```text
enp2s0           UP  10.21.31.192/24
wlx6c1ff78bc632  UP  192.168.112.146/21
```

绑定 `10.21.31.192` 后，外接主机搜索雷达相关话题输出为：

```text
/LIDAR/IMU201
/LIDAR/IMU202
/LIDAR/STATUS
```

没有 `/LIDAR/POINTS` 和 `/LIDAR/POINTS2`。

这说明：

```text
1. 外接主机 ROS2/FastDDS 基础通信是通的。
2. 外接主机网卡绑定到 10.21.31.192 后仍能发现部分 LIDAR 状态话题。
3. /LIDAR/POINTS 点云没有被发布或转发到 10.21.31 外接主机可发现的 DDS 网络中。
4. 当前问题不再只是外接主机选错 192.168.112.146 网卡。
```

当前最可疑原因：

```text
AOS 103 上 /opt/robot/scripts/setup_ros2.sh 选择的是 10.21.33.103。
/LIDAR/POINTS 很可能绑定在 AOS/雷达的 10.21.33 网段。
外接背部主机只接入 10.21.31.192，无法直接发现或接收 10.21.33 网段上的高带宽点云。
NOS multicast-relay 可能没有正确把 10.21.33 的点云 DDS 流转发到 10.21.31。
```

下一步不要再反复改外接主机本地 FastDDS profile，优先做三项验证。

验证 1：外接主机能否到达 10.21.33 网段：

```bash
source ~/m20_ros_env.sh
ping -c 3 10.21.33.103
ping -c 3 10.21.33.106
```

判断：

```text
如果 ping 不通，外接主机没有 10.21.33 路由或物理链路。
此时 /LIDAR/POINTS 若只在 10.21.33 发布，外接主机自然收不到。
```

验证 2：NOS 106 的组播转发是否真的在转发 10.21.33 到 10.21.31：

```bash
ssh user@10.21.31.106
sudo systemctl status multicast-relay.service
sudo journalctl -u multicast-relay.service -n 80 --no-pager
ip -br addr
```

验证 3：AOS root 下查看 `/LIDAR/POINTS` 发布者和 QoS：

```bash
ssh user@10.21.31.103
su
source /opt/robot/scripts/setup_ros2.sh
ros2 topic info -v /LIDAR/POINTS
ros2 topic hz /LIDAR/POINTS
```

如果 AOS root 可以稳定收到 9Hz，而外接主机只能看到 `/LIDAR/STATUS`，工程上有三条路线：

```text
路线 A：让外接主机接入或路由到 10.21.33 网段，再用 FastDDS whitelist 绑定对应网卡。
路线 B：修 NOS multicast-relay，让 10.21.33 点云 DDS 正确转发到 10.21.31。
路线 C：把 Lightning-LM 部署到机器狗内部推荐的 GOS 104 或可见点云的 AOS 103 上。
```

短期复现建图时，可以先在 AOS root 上录 bag：

```bash
ros2 bag record -o m20_lio_smoke /tf /IMU /LIDAR/POINTS
```

然后把 bag 拷到背部主机离线验证 Lightning-LM。这样能把“网络/DDS 可见性”和“算法编译/建图”两个问题拆开。

### 1.4.2 本轮新增验证结论

外接背部主机执行：

```bash
source ~/m20_ros_env.sh
ping -c 3 10.21.33.103
ping -c 3 10.21.33.106
```

结果：

```text
10.21.33.103: 100% packet loss
10.21.33.106: 100% packet loss
```

这证明外接背部主机 `10.21.31.192` 当前没有到 10.21.33 雷达网段的三层连通性。

NOS 106 上：

```bash
sudo systemctl status multicast-relay.service
ip -br addr
```

结果：

```text
multicast-relay.service active (running)
eth0  UP  10.21.33.106/24
eth1  UP  10.21.31.106/24
```

说明 NOS 确实同时连着 10.21.33 和 10.21.31 两个网段，组播转发服务也在运行。但是服务运行不等于 `/LIDAR/POINTS` 的 DDS 数据一定被完整转发到外接主机。

AOS 103 root 下：

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic info -v /LIDAR/POINTS
ros2 topic hz /LIDAR/POINTS
```

结果要点：

```text
Type: sensor_msgs/msg/PointCloud2
Publisher count: 2
Publisher node: _CREATED_BY_BARE_DDS_APP_
Reliability: RELIABLE
Durability: VOLATILE
average rate: about 10 Hz
```

这说明 `/LIDAR/POINTS` 不是普通 ROS2 节点发布，而是由 bare DDS 应用发布。AOS root 可以接收 10Hz 点云，但外接背部主机只能看到 `/LIDAR/IMU201`、`/LIDAR/IMU202`、`/LIDAR/STATUS`，看不到 `/LIDAR/POINTS`。

当前最可信判断：

```text
/LIDAR/POINTS 点云发布者工作正常。
点云发布侧在 AOS/雷达 10.21.33 网段。
外接背部主机没有 10.21.33 路由。
multicast-relay 虽然 active，但没有让外接主机发现或接收 /LIDAR/POINTS。
这已经不是 Lightning-LM 配置问题，也不是背部主机 ROS_DOMAIN_ID 或网卡白名单的单点问题。
```

下一步建议按优先级选路线。

路线 1，推荐工程验证路线：在 AOS 103 上录 bag，再拷到外接背部主机离线建图：

```bash
ssh user@10.21.31.103
su
source /opt/robot/scripts/setup_ros2.sh
ros2 bag record -o m20_lio_smoke /tf /IMU /LIDAR/POINTS
```

路线 2，网络打通路线：让外接背部主机能到达 10.21.33 网段，例如增加一条经 NOS 的路由：

```bash
sudo ip route add 10.21.33.0/24 via 10.21.31.106 dev enp2s0
ping -c 3 10.21.33.103
ping -c 3 10.21.33.106
```

这条路线是否可行取决于 NOS 是否启用了 IP forwarding 和是否允许转发。仅加外接主机路由不一定够。

在 NOS 上可检查：

```bash
cat /proc/sys/net/ipv4/ip_forward
```

如果为 `0`，说明 NOS 不做普通三层路由。是否允许临时开启需要谨慎评估，不建议未经确认直接改官方网络。

路线 3，官方推荐资源分配路线：把 Lightning-LM 部署到 GOS 104 或 AOS 103，而不是外接背部主机。官方资料也提示二次开发优先部署在 GOS 104，NOS 106 次之，尽量避免抢 AOS/NOS 核心进程资源。若目标是稳定拿实时点云，部署在机器狗内部可见 `/LIDAR/POINTS` 的主机上会少很多 DDS 跨网段问题。

### 1.4.3 GOS 104 新增验证结论

本轮也测试了官方推荐二次开发主机 GOS 104：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
ros2 topic list | grep -Ei "lidar|point|points|imu|odom"
ros2 topic hz /LIDAR/POINTS
ros2 topic hz /IMU
ros2 topic echo /ODOM --once
```

输出要点：

```text
Detected eth0 IP: 10.21.31.104
Selected IP(s): 10.21.31.104

可见话题:
/IMU
/LIDAR/IMU201
/LIDAR/IMU202
/LIDAR/STATUS
/ODOM
/UWB_ODOM_LOST

/LIDAR/POINTS:
WARNING: topic [/LIDAR/POINTS] does not appear to be published yet

/IMU:
about 200 Hz
```

另外，GOS 104 上的 ROS2 Foxy CLI 不支持：

```bash
ros2 topic echo /ODOM --once
```

如果要只看一条消息，可用：

```bash
timeout 3 ros2 topic echo /ODOM
```

或：

```bash
ros2 topic echo /ODOM | head -n 30
```

这轮结论：

```text
GOS 104 目前也看不到 /LIDAR/POINTS。
GOS 104 能看到 /IMU 和 /ODOM，说明基础 DDS 通信正常。
实时点云仍然被限制在 AOS/10.21.33 雷达链路，或没有通过现有 relay 转发到 GOS/10.21.31。
```

因此，导航部署路线进一步收敛：

```text
1. 如果要使用 /LIDAR/POINTS 做三维避障，必须先解决点云从 AOS/10.21.33 到 GOS/外接主机的实时转发。
2. GOS 104 适合跑二次开发程序，但当前不能直接跑依赖 /LIDAR/POINTS 的实时三维导航。
3. AOS 103 root 当前是唯一已确认能稳定收到 /LIDAR/POINTS 10Hz 的主机。
```

下一步优先确认 GOS 104 是否能到达雷达网段：

```bash
ssh user@10.21.31.104
ip -br addr
ip route
ping -c 3 10.21.33.103
ping -c 3 10.21.33.106
```

如果 GOS 104 也 ping 不通 10.21.33，则不能把 Lightning-LM/Nav2/SCAN 直接放在 GOS 上等它自动拿到点云。需要做网络路由、DDS relay 或 AOS/GOS 点云桥接。

### 1.4.4 只配置外接背部主机的路由验证结论

本轮按“暂不动机器狗内部主机，只改外接背部主机”的原则，在背部主机执行：

```bash
source ~/m20_ros_env.sh
ip -br addr
ip route
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY
echo $RMW_IMPLEMENTATION
echo $FASTRTPS_DEFAULT_PROFILES_FILE
ros2 topic list | grep -Ei "lidar|point|points|imu|odom"
sudo ip route add 10.21.33.0/24 via 10.21.31.106 dev enp2s0
```

实测环境：

```text
enp2s0: 10.21.31.192/24
wlx6c1ff78bc632: 192.168.112.146/21
ROS_DOMAIN_ID=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTRTPS_DEFAULT_PROFILES_FILE=/home/m20/.ros/m20_fastdds.xml
```

路由结果：

```text
10.21.33.0/24 via 10.21.31.106 dev enp2s0
```

ping 结果：

```text
ping 10.21.33.106: 通，0% 丢包
ping 10.21.33.103: 不通，100% 丢包
```

话题结果：

```text
可见:
/IMU
/ODOM
/LIDAR/IMU201
/LIDAR/IMU202
/LIDAR/STATUS
/LOC_BODY_POINTS

不可见或无数据:
/LIDAR/POINTS
```

这轮结论：

```text
1. 外接背部主机加静态路由后，只能到达 NOS 自己的 10.21.33.106。
2. 不能到达 AOS 的 10.21.33.103，说明 NOS 没有把外接主机流量继续转发到 10.21.33 网段内其它主机，或被防火墙/路由策略拦住。
3. 因此，仅配置外接背部主机路由，还不能解决 /LIDAR/POINTS 实时接入。
4. /LOC_BODY_POINTS 已经能在外接背部主机被发现，可能是官方链路转出的处理后点云，值得单独确认它能否作为导航感知输入。
```

只在外接背部主机继续做的下一步非侵入式检查：

```bash
source ~/m20_ros_env.sh
ros2 topic type /LOC_BODY_POINTS
ros2 topic info -v /LOC_BODY_POINTS
ros2 topic hz /LOC_BODY_POINTS
timeout 3 ros2 topic echo /LOC_BODY_POINTS
```

本轮已验证 `/LOC_BODY_POINTS`：

```text
type: sensor_msgs/msg/PointCloud2
Publisher count: 0
Subscription count: 1
Subscriber: accumulate_cloud
hz: 无数据
echo: 无数据
```

所以 `/LOC_BODY_POINTS` 当前不是可用点云输入。它只是某个官方节点的订阅入口，背部主机上没有发现对应发布者。

如果未来 `/LOC_BODY_POINTS` 有发布者且频率稳定，它可能先用于：

```text
点云转 /scan
Nav2 2D 避障验证
局部障碍物感知测试
```

但它不一定等价于原始双 RoboSense `/LIDAR/POINTS`。在用于三维建图或 Lightning-LM 前，需要确认字段、坐标系、是否已经去畸变、是否已经过滤动态/地面点。

继续只在外接背部主机上排查时，下一步应枚举所有 `PointCloud2` 话题，找“有 publisher 且有频率”的话题：

```bash
source ~/m20_ros_env.sh
for t in $(ros2 topic list); do
  ty=$(ros2 topic type "$t" 2>/dev/null)
  if [ "$ty" = "sensor_msgs/msg/PointCloud2" ]; then
    echo "===== $t ====="
    ros2 topic info -v "$t" | sed -n '1,35p'
  fi
done
```

对有 `Publisher count > 0` 的 `PointCloud2` 话题，再测频率：

```bash
ros2 topic hz <pointcloud_topic>
timeout 3 ros2 topic echo <pointcloud_topic>
```

本轮继续枚举外接背部主机可见的全部 `PointCloud2` 后，结果变成：

```text
/LOC_BODY_POINTS                 PointCloud2，Publisher count 0，只有 accumulate_cloud 订阅
/accumulate_cloud/cloud_base      PointCloud2，Publisher count 1，发布者 accumulate_cloud
/accumulate_cloud/cloud_gravity   PointCloud2，Publisher count 1，发布者 accumulate_cloud，passable_area 订阅
/passable_area                    PointCloud2，Publisher count 1，发布者 passable_area
/impassable_area                  PointCloud2，Publisher count 1，发布者 passable_area
```

这说明外接背部主机仍然收不到原始 `/LIDAR/POINTS`，但已经能看到官方链路处理后的点云输出。这个结论很关键：

```text
Lightning-LM / FAST-LIO 类建图定位:
  仍优先需要原始 /LIDAR/POINTS。
  /accumulate_cloud 或 passable_area 输出大概率已经被累计、重力对齐、地面/障碍分割处理过，
  不应直接当作原始双 RoboSense 点云使用，除非字段和时间戳确认满足算法要求。

Nav2 / MPPI / 2D 避障:
  /impassable_area 或 /accumulate_cloud/cloud_base 可能已经足够作为障碍物输入。
  下一步应验证频率、frame_id、fields、点云高度范围，再决定转 /scan 还是直接进 costmap voxel/obstacle layer。

三维局部避障 / m20_scan_planner:
  /accumulate_cloud/cloud_gravity 可能比原始点云更接近局部三维环境输入。
  但要确认它是 robot/map/odom 哪个坐标系、是否有明显延迟、是否随机器人运动连续更新。
```

只在外接背部主机上继续验证这些处理后点云：

```bash
source ~/m20_ros_env.sh

ros2 topic hz /accumulate_cloud/cloud_base
ros2 topic hz /accumulate_cloud/cloud_gravity
ros2 topic hz /passable_area
ros2 topic hz /impassable_area
```

查看每个话题的 `header.frame_id`、`height`、`width`、`fields`：

```bash
timeout 3 ros2 topic echo /accumulate_cloud/cloud_base | sed -n '1,120p'
timeout 3 ros2 topic echo /accumulate_cloud/cloud_gravity | sed -n '1,120p'
timeout 3 ros2 topic echo /passable_area | sed -n '1,120p'
timeout 3 ros2 topic echo /impassable_area | sed -n '1,120p'
```

本轮频率和 echo 验证结果：

```text
timeout 8 ros2 topic hz /accumulate_cloud/cloud_base      无输出
timeout 8 ros2 topic hz /accumulate_cloud/cloud_gravity   无输出
timeout 8 ros2 topic hz /passable_area                    无输出
timeout 8 ros2 topic hz /impassable_area                  无输出

timeout 3 ros2 topic echo 上述 4 个话题均无消息内容，仅超时结束
```

因此当前更准确的结论是：

```text
这些 PointCloud2 话题在 DDS 图里有 publisher 端点；
但当前没有实时样本流，不能直接作为导航输入；
ros2 topic info 能看到 Publisher count 1，不代表该 topic 正在发布数据。
```

结合 `/LOC_BODY_POINTS` 没有发布者这一点，推测链路是：

```text
/LOC_BODY_POINTS 无上游发布者
  -> accumulate_cloud 没有输入
  -> /accumulate_cloud/cloud_base 和 /accumulate_cloud/cloud_gravity 没有实际样本
  -> passable_area 没有输入
  -> /passable_area 和 /impassable_area 没有实际样本
```

本轮继续查看节点图后，这条链路得到确认：

```text
/accumulate_cloud
  Subscribers:
    /IMU
    /LOC_BODY_POINTS
    /ODOM
    /PASSABLE_AREA_ENABLE
  Publishers:
    /accumulate_cloud/cloud_base
    /accumulate_cloud/cloud_gravity
    /accumulate_cloud/status_code
    /tf

/passable_area
  Subscribers:
    /IMU
    /accumulate_cloud/cloud_gravity
  Publishers:
    /grid_map
    /impassable_area
    /passable_area
    /passable_status_code
    /traversal_cost
```

所以当前问题先定位到两个条件：

```text
1. /accumulate_cloud 的输入话题配置是 /LOC_BODY_POINTS。
2. /accumulate_cloud 默认 wait_for_switch_flag=true，需要 /PASSABLE_AREA_ENABLE 使能。
```

下一步只在外接背部主机继续做非侵入式检查：

```bash
source ~/m20_ros_env.sh

ros2 param get /accumulate_cloud cloud_topic
ros2 param get /accumulate_cloud cloud_base_pub_topic
ros2 param get /accumulate_cloud cloud_gravity_pub_topic
ros2 param get /accumulate_cloud wait_for_switch_flag
ros2 param get /accumulate_cloud switch_flag_topic

ros2 param get /passable_area accumulate_cloud_topic
ros2 param get /passable_area passable_cloud_topic
ros2 param get /passable_area impassable_cloud_topic
ros2 param get /passable_area traversal_cost_topic
```

再看状态码是否提示“未使能”或“无输入”：

```bash
timeout 5 ros2 topic echo /accumulate_cloud/status_code
timeout 5 ros2 topic echo /passable_status_code
```

如果状态码显示只是等待 `/PASSABLE_AREA_ENABLE`，再考虑临时发一次使能：

```bash
ros2 topic pub --once /PASSABLE_AREA_ENABLE std_msgs/msg/Int32 "{data: 1}"
```

本轮实测参数和使能结果：

```text
/accumulate_cloud cloud_topic          = /LOC_BODY_POINTS
/accumulate_cloud wait_for_switch_flag = true
/accumulate_cloud switch_flag_topic    = /PASSABLE_AREA_ENABLE
/accumulate_cloud odom_topic           = /ODOM
/accumulate_cloud imu_topic            = /IMU

/passable_area accumulate_cloud_topic  = /accumulate_cloud/cloud_gravity
/passable_area passable_cloud_topic    = /passable_area
/passable_area impassable_cloud_topic  = /impassable_area
/passable_area traversal_cost_topic    = /traversal_cost

/accumulate_cloud/status_code          = 101，持续输出
/IMU                                  ~= 200Hz
/ODOM                                 ~= 10Hz
/LOC_BODY_POINTS                       hz 无输出
```

向 `/PASSABLE_AREA_ENABLE` 发送一次 `{data: 1}` 后，处理后点云开始输出：

```text
/accumulate_cloud/cloud_gravity ~= 1.9 - 2.2Hz
/impassable_area                ~= 1.7 - 1.8Hz
```

再次使能并完整测频后，官方处理后链路稳定输出：

```text
/accumulate_cloud/cloud_base    ~= 1.43Hz
/accumulate_cloud/cloud_gravity ~= 1.43Hz
/passable_area                  ~= 1.43Hz
/impassable_area                ~= 1.43Hz
/traversal_cost                 ~= 1.43Hz
/passable_status_code           = 100
```

这个结果说明：

```text
之前处理后点云没有数据，至少有一个原因是未发送 /PASSABLE_AREA_ENABLE。
status_code=101 大概率表示等待使能或未处于工作状态，但具体含义仍需官方文档/源码确认。
即使 /LOC_BODY_POINTS 的 hz 没有输出，下游在使能后仍能产生处理后点云；因此不能再简单判断“无 LOC_BODY_POINTS 就完全不可用”。
passable_status_code=100 后，/passable_area、/impassable_area、/traversal_cost 已有稳定低频输出，具备接入导航前验证价值。
```

下一步必须确认使能后的点云内容是否真的可用于导航：

```bash
source ~/m20_ros_env.sh

ros2 topic pub --once /PASSABLE_AREA_ENABLE std_msgs/msg/Int32 "{data: 1}"

timeout 8 ros2 topic hz /accumulate_cloud/cloud_base
timeout 8 ros2 topic hz /accumulate_cloud/cloud_gravity
timeout 8 ros2 topic hz /passable_area
timeout 8 ros2 topic hz /impassable_area
timeout 8 ros2 topic hz /traversal_cost

timeout 3 ros2 topic echo /accumulate_cloud/cloud_gravity | sed -n '1,120p'
timeout 3 ros2 topic echo /impassable_area | sed -n '1,120p'
timeout 3 ros2 topic echo /traversal_cost | sed -n '1,120p'
timeout 3 ros2 topic echo /accumulate_cloud/status_code
timeout 3 ros2 topic echo /passable_status_code
```

注意：`PointCloud2` 和 `OccupancyGrid` 消息可能很大，`timeout 3 ros2 topic echo <topic> | sed -n '1,120p'` 可能在还没打印完一帧时就被 timeout 杀掉。`hz` 已经证明有数据时，`echo` 无输出不等价于 topic 无数据。

优先尝试只打印元数据：

```bash
ros2 topic echo -h | grep -E -- '--no-arr|--field'

timeout 8 ros2 topic echo /accumulate_cloud/cloud_gravity --no-arr
timeout 8 ros2 topic echo /impassable_area --no-arr
timeout 8 ros2 topic echo /traversal_cost --no-arr
```

本轮确认 Foxy 支持 `--no-arr`，并拿到关键元数据：

```text
/accumulate_cloud/cloud_gravity:
  frame_id: base_gravity
  height: 1
  width: 约 11700 - 12000 点
  fields: 8 个字段
  point_step: 48
  is_dense: true

/impassable_area:
  frame_id: base_gravity
  height: 1
  width: 约 4700 - 5100 点
  fields: 3 个字段
  point_step: 16
  is_dense: true

/traversal_cost:
  type: nav_msgs/msg/OccupancyGrid
  frame_id: base_gravity
  resolution: 0.05m
  width: 160
  height: 160
  origin: x=-4.0, y=-4.0, z=0.0
  data_len: 25600
```

这表示 `/traversal_cost` 是一个以 `base_gravity` 为坐标系的 8m x 8m 局部代价图，分辨率 5cm，范围大致是机器人周围 `[-4m, +4m]`。它非常适合作为 Nav2/MPPI 局部避障的候选输入，但还要确认 TF 和 RViz 显示。

下一步优先验证 TF：

```bash
source ~/m20_ros_env.sh

timeout 5 ros2 topic hz /tf
timeout 5 ros2 topic echo /tf --no-arr
ros2 run tf2_ros tf2_echo odom base_gravity
ros2 run tf2_ros tf2_echo base_gravity base_link
```

本轮 TF 结果：

```text
/tf 约 11.3 - 11.7Hz
tf2_echo odom base_gravity 失败：frame "odom" 不存在

/accumulate_cloud body_frame          = base_link
/accumulate_cloud base_gravity_frame  = base_gravity
/passable_area body_frame             = base_link
/passable_area gravity_frame          = base_gravity
```

这说明感知链路自己的坐标系是 `base_gravity` / `base_link`，但当前 ROS 图里不一定存在名为 `odom` 的 TF frame。`tf2_echo odom base_gravity` 失败不代表 `/traversal_cost` 或点云不可用；它只是说明后续接 Nav2 时，必须额外确认或补齐 odom/local frame 到 robot frame 的 TF。

`ros2 topic echo /tf --no-arr` 会把 transforms 数组隐藏掉，无法看到真实 parent/child frame。用下面的小脚本打印 TF 边：

```bash
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

class TFProbe(Node):
    def __init__(self):
        super().__init__('m20_tf_probe')
        self.create_subscription(TFMessage, '/tf', self.cb, 10)
        self.seen = set()

    def cb(self, msg):
        for t in msg.transforms:
            key = (t.header.frame_id, t.child_frame_id)
            if key not in self.seen:
                self.seen.add(key)
                tr = t.transform.translation
                q = t.transform.rotation
                print(f'{t.header.frame_id} -> {t.child_frame_id}: xyz=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f}) q=({q.x:.3f},{q.y:.3f},{q.z:.3f},{q.w:.3f})', flush=True)

rclpy.init()
node = TFProbe()
end = time.time() + 8.0
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
PY
```

重点确认是否存在：

```text
base_gravity -> base_link
某个里程计/世界 frame -> base_gravity 或 base_link
```

如果 `base_link` 名字不对，先查 TF 里真实的 body frame：

```bash
timeout 5 ros2 topic echo /tf --no-arr
ros2 param get /accumulate_cloud body_frame
ros2 param get /accumulate_cloud base_gravity_frame
ros2 param get /passable_area body_frame
ros2 param get /passable_area gravity_frame
```

RViz 验证建议：

```text
Fixed Frame: base_gravity
Add -> By topic -> /traversal_cost OccupancyGrid
Add -> By topic -> /impassable_area PointCloud2
Add -> By topic -> /passable_area PointCloud2
Add -> By topic -> /accumulate_cloud/cloud_gravity PointCloud2
```

本轮 RViz 截图已经确认：在 `base_gravity` 下，点云可以显示，说明背部主机至少能可视化官方处理后的局部感知结果。下一步还需要确认 `/traversal_cost` 的 OccupancyGrid 是否也能正常显示，以及画面中的障碍点是否和现场物体一致。

如果 RViz 中 `/traversal_cost` 随机器人运动稳定更新，且 `/impassable_area` 能覆盖真实障碍物，则优先走：

```text
/impassable_area -> Nav2 obstacle/voxel layer
或 /traversal_cost -> 自定义 costmap layer / 代价图桥接节点
```

如果当前 Foxy 的 `ros2 topic echo` 不支持 `--no-arr`，用一个最小 Python 订阅脚本只打印 header 和尺寸：

```bash
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid

class Probe(Node):
    def __init__(self):
        super().__init__('m20_topic_meta_probe')
        self.create_subscription(PointCloud2, '/accumulate_cloud/cloud_gravity', lambda msg: self.pc_cb('/accumulate_cloud/cloud_gravity', msg), 10)
        self.create_subscription(PointCloud2, '/impassable_area', lambda msg: self.pc_cb('/impassable_area', msg), 10)
        self.create_subscription(OccupancyGrid, '/traversal_cost', self.grid_cb, 10)

    def pc_cb(self, topic, msg):
        fields = ','.join(f.name for f in msg.fields)
        print(f'[{topic}] frame={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} width={msg.width} height={msg.height} point_step={msg.point_step} fields={fields}', flush=True)

    def grid_cb(self, msg):
        info = msg.info
        print(f'[/traversal_cost] frame={msg.header.frame_id} stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d} size={info.width}x{info.height} res={info.resolution} origin=({info.origin.position.x},{info.origin.position.y},{info.origin.position.z}) data_len={len(msg.data)}', flush=True)

rclpy.init()
node = Probe()
end = time.time() + 8.0
while rclpy.ok() and time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
PY
```

重点看：

```text
header.frame_id
height / width
fields
point_step
OccupancyGrid 的 resolution / width / height / origin
```

为了后续离线看 RViz 和写转换节点，建议录一小段 bag：

```bash
mkdir -p ~/m20_topic_probe
ros2 topic pub --once /PASSABLE_AREA_ENABLE std_msgs/msg/Int32 "{data: 1}"
ros2 bag record -o ~/m20_topic_probe/processed_clouds \
  /IMU /ODOM \
  /accumulate_cloud/cloud_base \
  /accumulate_cloud/cloud_gravity \
  /passable_area /impassable_area /traversal_cost
```

当前 `ros2 topic hz` 已经确认处理后点云和 `/traversal_cost` 有稳定低频输出，背部主机可以继续走“官方处理点云 -> 导航避障”的验证路线。如果要做 Lightning-LM 原始建图定位，仍需要继续解决 `/LIDAR/POINTS` 跨网段/DDS 可见性。

如果坚持使用原始 `/LIDAR/POINTS` 做实时导航，仅靠外接背部主机配置已经不够。后续需要：

```text
让 NOS 开启/允许 10.21.31 -> 10.21.33 转发；
或配置官方 multicast/DDS relay 转发 /LIDAR/POINTS；
或在 AOS/GOS 做点云桥接；
或把依赖原始点云的算法部署到 AOS 侧并严格限资源。
```

### 1.4.5 官方 relay 原装方法复测结论

本轮按“先试官方原装方法，不先上自研桥接”的思路复测：

```bash
# NOS 106
sudo systemctl restart multicast-relay.service
sudo systemctl status multicast-relay.service
sudo journalctl -u multicast-relay.service -n 80 --no-pager

# AOS 103 root
source /opt/robot/scripts/setup_ros2.sh
ros2 topic info -v /LIDAR/POINTS
ros2 topic hz /LIDAR/POINTS

# GOS 104
source /opt/robot/scripts/setup_ros2.sh
ros2 topic list | grep -Ei "LIDAR|POINT|LOC|cloud|passable|traversal"
ros2 topic info -v /LIDAR/POINTS
ros2 topic hz /LIDAR/POINTS
ros2 topic hz /LIDAR/POINTS2
ros2 topic hz /LOC_BODY_POINTS
```

实测结果：

```text
NOS 106:
  multicast-relay.service 已 enabled 且 active (running)
  日志只有 stop/start，没有显示具体 topic 级转发配置

AOS 103 root:
  /opt/robot/scripts/setup_ros2.sh 选择 10.21.33.103
  /LIDAR/POINTS publisher count = 2
  发布者为 _CREATED_BY_BARE_DDS_APP_
  QoS 为 RELIABLE + VOLATILE
  ros2 topic hz /LIDAR/POINTS 稳定约 9.10Hz

GOS 104:
  /opt/robot/scripts/setup_ros2.sh 选择 10.21.31.104
  可见 /LIDAR/IMU201、/LIDAR/IMU202、/LIDAR/STATUS
  可见 /LOCATION_STATUS、/PASSABLE_AREA_ENABLE、/accumulate_cloud/status_code、/passable_status_code
  ros2 topic info -v /LIDAR/POINTS 返回 Unknown topic
  /LIDAR/POINTS、/LIDAR/POINTS2、/LOC_BODY_POINTS 均无 hz 输出
```

这个结果说明：

```text
1. 官方 multicast-relay 服务本身运行正常。
2. AOS 上原始 /LIDAR/POINTS 发布正常，约 9 - 10Hz。
3. GOS 上完全发现不到 /LIDAR/POINTS，连 topic info 都是 Unknown topic。
4. 因此当前不是 QoS 不匹配或 ros2 topic hz 用法问题，而是原始点云没有被官方默认链路转发到 GOS。
5. 官方默认暴露到 10.21.31 业务网的更像是状态、IMU/ODOM、位置状态和可通行区域链路的一部分，不是原始双雷达点云全量透传。
```

所以当前路线排序调整为：

```text
第一优先:
  如果能从官方拿到 /LIDAR/POINTS 跨 31/33 网段的 relay/topic 白名单配置，就用官方配置。

第二优先:
  使用 AOS 轻量桥接，只转 ROI/降采样后的障碍云到 GOS。

第三优先:
  把依赖原始点云的建图/定位先放在 AOS root 限核运行，GOS/背部主机只接结果。
```

在没有官方原始点云转发配置前，不要继续假设 GOS 能通过 `multicast-relay.service` 自动看到 `/LIDAR/POINTS`。

### 1.4.6 更方便路线：本地解码官方转发的雷达 UDP 组播

本轮继续查看 NOS 106 上的官方 `multicast-relay.service` 后，发现它不是 ROS2/DDS topic relay，而是一个很直接的 UDP 组播转发脚本：

```text
systemd:
  ExecStart=/usr/bin/python3 /usr/bin/multicast.py

/usr/bin/multicast.py:
  GROUPS = [
      ("224.10.10.202", 6692),
      ("224.10.10.202", 7782),
      ("224.10.10.201", 6691),
      ("224.10.10.201", 7781),
  ]

  RECV_IFACE = "10.21.33.106"
  SEND_IFACE = "10.21.31.106"
```

这解释了之前的矛盾：

```text
官方 relay 没有把 ROS2 /LIDAR/POINTS topic 转到 GOS；
但它可能已经把前/后雷达的原始 UDP 组播包从 33 雷达网转到了 31 业务网。
```

因此，比 AOS 自研桥接更“原装”的下一步是：

```text
NOS 106:
  继续运行 multicast.py，把两颗雷达 UDP 组播包转到 10.21.31 网段。

GOS 104 或外接背部主机:
  本地运行 RoboSense rslidar_sdk，加入 224.10.10.201/202 组播组，
  监听 6691/7781、6692/7782，
  自己发布 PointCloud2。
```

这条路线的本质不是“订阅官方 `/LIDAR/POINTS`”，而是“复用官方 UDP 组播转发，在导航主机本地生成新的点云 topic”。

#### 步骤 1：确认 31 网段能收到雷达 UDP 组播包

在 GOS 104：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh

ip -br addr
timeout 10 sudo tcpdump -ni eth0 \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 7781 or port 6692 or port 7782)' \
  -c 30
```

在外接背部主机，如果网线口是 `enp2s0`：

```bash
source ~/m20_ros_env.sh

ip -br addr
timeout 10 sudo tcpdump -ni enp2s0 \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 7781 or port 6692 or port 7782)' \
  -c 30
```

判断：

```text
能抓到 UDP 包:
  说明官方 multicast.py 确实把雷达原始包转到了 31 业务网；
  下一步可以用 rslidar_sdk 在 GOS/背部主机本地解码出 PointCloud2。

抓不到 UDP 包:
  说明 multicast.py 转发没有到达当前主机或交换机/NIC 没有放行；
  此时继续调 ROS2 topic 没意义，应先解决 UDP 组播可达性。
```

本轮 GOS 104 已经实测抓到 UDP 包：

```text
10.21.31.106 -> 224.10.10.202:6692 UDP length 1248
10.21.31.106 -> 224.10.10.201:6691 UDP length 1248
30 packets captured
0 packets dropped by kernel
```

这说明：

```text
1. NOS 106 的 multicast.py 确实在把两路雷达 MSOP 包转发到 10.21.31 业务网。
2. GOS 104 可以从 eth0 收到这些包。
3. 当前缺的不是网络转发，而是在 GOS/背部主机上运行 rslidar_sdk 解码这些 UDP 包。
4. 这条路线比 AOS 自研 ROS2 点云桥接更接近官方原装链路。
```

背部主机本轮 tcpdump 失败是因为网卡名写成了 `eth0`：

```text
tcpdump: eth0: No such device exists
```

背部主机之前实测的机器狗网线口是 `enp2s0`，应改用：

```bash
timeout 10 sudo tcpdump -ni enp2s0 \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 7781 or port 6692 or port 7782)' \
  -c 30
```

注意：本轮只在前 30 个包里抓到了 `6691/6692` 的 MSOP 包，没有抓到 `7781/7782` 的 DIFOP 包。DIFOP 频率可能低，也可能没有被当前时长捕获。正式解码前建议单独抓久一点：

```bash
timeout 60 sudo tcpdump -ni eth0 \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 7781 or port 7782)' \
  -c 10
```

本轮已经继续确认：

```text
GOS 104:
  7781/7782 DIFOP 包也能抓到，约 1Hz：
    10.21.31.106 -> 224.10.10.201:7781 UDP length 1248
    10.21.31.106 -> 224.10.10.202:7782 UDP length 1248

  MSOP 包头：
    55 aa 05 5a ...
    payload 约第 42 字节处出现 ff ee block id

  GOS 上没有现成 rslidar_sdk：
    ros2 pkg prefix rslidar_sdk -> Package not found

外接背部主机:
  使用 enp2s0 后，也能抓到 6691/6692 MSOP 包。
```

结合本地 `rslidar_sdk` 源码，`1248` 长度、`55 aa 05 5a` 包头、`ff ee` block id 更吻合 `RSHELIOS` decoder；`RSM1` 的 MSOP 长度是 `1210`，不适合作为当前首选。

#### 步骤 2：确认官方 rslidar 的实际型号配置

`rslidar_sdk` 需要准确的 `lidar_type`。当前抓包特征更像 `RSHELIOS`，但 M20 实机两颗雷达的官方型号仍应优先从机器狗官方 `rslidar` 进程配置里确认。

本轮抓到的 MSOP UDP 包长度是 `1248`，而本地 `rslidar_sdk` 中 `RSM1` 的 MSOP 长度是 `1210`。因此，之前示例里的 `RSM1` 不适合作为当前首选。`RSHELIOS` 的特征是：

```text
msop len: 1248
difop len: 1248
msop id: 55 aa 05 5a
block id: ff ee
```

这与当前抓包结果吻合。下一步可以先用 `RSHELIOS` 做 smoke test；如果不出点云，再继续查官方配置或尝试其它 1248 长度 decoder。

在 AOS/GOS/NOS 任一正在运行 `./rslidar` 的主机上：

```bash
ps -ef | grep -Ei "rslidar|hsLidar" | grep -v grep

PID=$(pgrep -f './rslidar' | head -n 1)
echo $PID
tr '\0' ' ' < /proc/$PID/cmdline
readlink -f /proc/$PID/cwd

sudo find /opt/robot /var/opt/robot /userdata -maxdepth 6 \
  \( -iname '*rslidar*' -o -iname '*lidar*.yaml' -o -iname '*lidar*.json' -o -iname '*lidar*.conf' \) \
  -print
```

重点找：

```text
lidar_type
msop_port / difop_port
group_address / host_address
frame_id
topic 名
```

如果暂时找不到官方配置，先不要盲目固定为 `RSM1`。建议再抓一帧包头，用于和 RoboSense SDK 的 decoder 匹配：

```bash
timeout 5 sudo tcpdump -ni eth0 -s 128 -XX \
  'udp and host 224.10.10.201 and port 6691' \
  -c 1

timeout 5 sudo tcpdump -ni eth0 -s 128 -XX \
  'udp and host 224.10.10.202 and port 6692' \
  -c 1
```

同时检查 GOS 是否已经有官方 `rslidar_sdk` 包：

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 pkg prefix rslidar_sdk
ros2 pkg executables rslidar_sdk
```

#### 步骤 3：在 GOS/背部主机用 rslidar_sdk 解码

仓库中已经新增两份 smoke test 配置：

```text
src/m20_lidar_bridge/config/rslidar_multicast_rshelios_gos.yaml
src/m20_lidar_bridge/config/rslidar_multicast_rshelios_backpack.yaml
```

其中使用：

```text
lidar_type: RSHELIOS
224.10.10.201:6691/7781 -> /m20/front/points
224.10.10.202:6692/7782 -> /m20/rear/points
```

如果需要现场手写配置，下面示例以 GOS 104 为例，`host_address` 使用 `10.21.31.104`；如果在背部主机跑，改成 `10.21.31.192`。

```bash
cat > /tmp/m20_rslidar_multicast.yaml <<'EOF'
common:
  msg_source: 1
  send_packet_ros: false
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RSHELIOS
      group_address: 224.10.10.201
      host_address: 10.21.31.104
      msop_port: 6691
      difop_port: 7781
      imu_port: 0
      min_distance: 0.2
      max_distance: 100
      use_lidar_clock: true
      dense_points: false
      wait_for_difop: false
      ts_first_point: true
      start_angle: 0
      end_angle: 360
    ros:
      ros_frame_id: front_lidar
      ros_recv_packet_topic: /m20/front/rslidar_packets
      ros_send_packet_topic: /m20/front/rslidar_packets
      ros_send_imu_data_topic: /m20/front/rslidar_imu
      ros_send_point_cloud_topic: /m20/front/points
      ros_queue_length: 20

  - driver:
      lidar_type: RSHELIOS
      group_address: 224.10.10.202
      host_address: 10.21.31.104
      msop_port: 6692
      difop_port: 7782
      imu_port: 0
      min_distance: 0.2
      max_distance: 100
      use_lidar_clock: true
      dense_points: false
      wait_for_difop: false
      ts_first_point: true
      start_angle: 0
      end_angle: 360
    ros:
      ros_frame_id: rear_lidar
      ros_recv_packet_topic: /m20/rear/rslidar_packets
      ros_send_packet_topic: /m20/rear/rslidar_packets
      ros_send_imu_data_topic: /m20/rear/rslidar_imu
      ros_send_point_cloud_topic: /m20/rear/points
      ros_queue_length: 20
EOF
```

如果在 GOS 上已经安装/编译了 `rslidar_sdk`：

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:=/tmp/m20_rslidar_multicast.yaml
```

当前 GOS 104 实测没有安装 `rslidar_sdk`，因此要么先把 `src/third_party/rslidar_sdk` 和 `src/third_party/rslidar_msg` 部署/编译到 GOS，要么先在外接背部主机做解码 smoke test。

如果在外接背部主机上用本工作区的 `rslidar_sdk`：

```bash
cd ~/robodog_nav_system
source install/setup.bash
ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:=$PWD/src/m20_lidar_bridge/config/rslidar_multicast_rshelios_backpack.yaml
```

当前本工作区已经能找到：

```text
ros2 pkg prefix rslidar_sdk -> install/rslidar_sdk
ros2 pkg executables rslidar_sdk -> rslidar_sdk rslidar_sdk_node
```

另开终端验证：

```bash
ros2 topic list | grep -Ei "m20/.*/points|rslidar|points"
ros2 topic hz /m20/front/points
ros2 topic hz /m20/rear/points
timeout 5 ros2 topic echo /m20/front/points --no-arr
timeout 5 ros2 topic echo /m20/rear/points --no-arr
```

如果能出点云，这条路线优先级高于 AOS 自研桥接：

```text
优点:
  不改 AOS；
  不抢 AOS 点云/LIO 资源；
  使用 RoboSense 官方 SDK 解码；
  GOS/背部主机可以直接拿本地点云做 Nav2、MPPI、Lightning-LM 或 SCAN 输入前处理。

注意:
  要确认 lidar_type 正确；
  要确认前/后雷达外参和 TF；
  wait_for_difop=false 只适合 smoke test，正式使用应优先改回 true 并确认 DIFOP 包稳定；
  两颗雷达点云需要后续合并、裁剪或分别接入 costmap。
```

如果 UDP 能抓到，但 rslidar_sdk 没有点云，优先排查：

```text
1. lidar_type 是否与实机一致；
2. host_address 是否写成当前主机 31 网段 IP；
3. group_address 与端口是否对应：
   前雷达可能是 224.10.10.201:6691/7781；
   后雷达可能是 224.10.10.202:6692/7782；
4. 是否需要 root 权限加入组播；
5. 是否已有官方 rslidar 进程占用了相同端口。
```

### 1.4.7 下一步：AOS 双网卡点云桥接验证

目标：验证 AOS 103 上的一个 ROS2 进程能否同时：

```text
从 10.21.33.103 雷达网段订阅 /LIDAR/POINTS
向 10.21.31.103 机器狗业务网段发布 /LIDAR/POINTS_BRIDGE
```

先不要启动导航，只验证话题可见性和频率。

当前仓库已经新增正式桥接包：

```text
src/m20_lidar_bridge
```

这个包里的 `pointcloud_roi_bridge` 不直接全量透传原始点云，而是在 AOS 上先做限频、ROI 裁剪、体素降采样和最大点数限制，再发布给 GOS：

```text
输入: /LIDAR/POINTS
输出: /m20/perception/obstacle_cloud
默认: max_rate_hz=10Hz, voxel_size=0.10m, max_points=30000
```

完整操作说明见：

```text
src/m20_lidar_bridge/docs/aos_to_gos_bridge.md
```

#### 步骤 1：AOS root 建立双网卡 FastDDS 配置

在 AOS 103 上：

```bash
ssh user@10.21.31.103
su
source /opt/robot/scripts/setup_ros2.sh

cat > /tmp/aos_dual_fastdds.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>aos_dual_udp</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>10.21.33.103</address>
        <address>10.21.31.103</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="aos_dual_participant" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>aos_dual_udp</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF

export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/aos_dual_fastdds.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
unset ROS_LOCALHOST_ONLY
```

确认在这个双网卡环境下仍能收到点云：

```bash
ros2 topic hz /LIDAR/POINTS
```

如果这里收不到 `/LIDAR/POINTS`，说明双网卡 profile 没能发现雷达侧 bare DDS 发布者，先停止，不要继续桥接。

#### 步骤 2：先发布一个低带宽测试话题

仍在 AOS 103 同一个终端：

```bash
ros2 topic pub /aos_dual_test std_msgs/msg/String "{data: aos_dual_ok}" -r 1
```

在外接背部主机上：

```bash
source ~/m20_ros_env.sh
ros2 topic echo /aos_dual_test
```

或在 GOS 104 上：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
ros2 topic echo /aos_dual_test
```

如果外接背部主机或 GOS 能收到 `/aos_dual_test`，说明 AOS 的双网卡 ROS2 participant 可以向 10.21.31 侧发消息。

#### 步骤 3：临时桥接 `/LIDAR/POINTS`

在 AOS 103 的双网卡环境终端中创建临时桥接脚本：

```bash
cat > /tmp/m20_pc2_relay.py <<'PY'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import PointCloud2

class PointCloudRelay(Node):
    def __init__(self):
        super().__init__("m20_pointcloud_relay")
        qos = QoSProfile(depth=5)
        self.count = 0
        self.pub = self.create_publisher(PointCloud2, "/LIDAR/POINTS_BRIDGE", qos)
        self.sub = self.create_subscription(PointCloud2, "/LIDAR/POINTS", self.callback, qos)
        self.get_logger().info("Relaying /LIDAR/POINTS -> /LIDAR/POINTS_BRIDGE")

    def callback(self, msg):
        self.count += 1
        self.pub.publish(msg)
        if self.count % 10 == 0:
            self.get_logger().info(f"relayed {self.count} point clouds")

def main():
    rclpy.init()
    node = PointCloudRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
PY

python3 /tmp/m20_pc2_relay.py
```

在外接背部主机上验证桥接点云：

```bash
source ~/m20_ros_env.sh
ros2 topic list | grep POINTS
ros2 topic info -v /LIDAR/POINTS_BRIDGE
ros2 topic hz /LIDAR/POINTS_BRIDGE
```

在 GOS 104 上也可验证：

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic hz /LIDAR/POINTS_BRIDGE
```

如果 `/LIDAR/POINTS_BRIDGE` 在外接背部主机或 GOS 上达到接近 10Hz，说明实时点云桥接可行。后续 Lightning-LM 或导航算法可先临时订阅：

```yaml
common:
  lidar_topic: "/LIDAR/POINTS_BRIDGE"
```

如果 `/LIDAR/POINTS_BRIDGE` 不可见或频率很低，说明 AOS 双网卡 DDS relay 方案不可直接用，需要改走系统级路由、官方 relay 配置，或把算法直接部署在可见点云的 AOS 侧并做资源隔离。

#### 步骤 4：使用正式 ROI/体素桥接包

临时 Python relay 只用于证明 DDS 链路可行，不建议长期全量透传原始点云。链路打通后，应使用 `m20_lidar_bridge`：

```bash
# 在 AOS 103 上准备工作空间，例如 ~/m20_lidar_bridge
cd ~/m20_lidar_bridge
source /opt/ros/foxy/setup.bash
colcon build --packages-select m20_lidar_bridge --symlink-install
```

启动时仍然需要 root 和双网卡 FastDDS profile：

```bash
su
cd ~/m20_lidar_bridge
source /opt/robot/scripts/setup_ros2.sh
source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/m20_lidar_bridge/install/m20_lidar_bridge/share/m20_lidar_bridge/dds/aos_dual_fastdds.xml

ros2 launch m20_lidar_bridge aos_roi_bridge.launch.py
```

GOS 104 验证：

```bash
source /opt/robot/scripts/setup_ros2.sh
ros2 topic info -v /m20/perception/obstacle_cloud
ros2 topic hz /m20/perception/obstacle_cloud
timeout 5 ros2 topic echo /m20/perception/obstacle_cloud --no-arr
```

只有当 `/m20/perception/obstacle_cloud` 在 GOS 上稳定达到 8 - 10Hz，并且 RViz 中点云覆盖真实障碍物，再开始接 Nav2/MPPI 或 m20_scan_planner。

### 1.5 检查 AOS 与 NOS 网络是否互通

在 AOS 上：

```bash
ping -c 3 10.21.31.106
ip addr
ip route
```

如果 ping 不通，优先处理网线、网卡地址、路由和 NetworkManager。DDS 话题发现依赖 UDP 组播，即使 ping 通，也可能因为组播转发、防火墙或网卡选择问题看不到话题。

临时排查防火墙：

```bash
sudo ufw status
```

如果现场允许，可以临时关闭后再测：

```bash
sudo ufw disable
```

### 1.6 用 ROS2 组播工具验证 DDS 发现

如果系统有 `ros2 multicast` 命令，可以两边互测。

终端 A：

```bash
source ~/m20_ros_env.sh
ros2 multicast receive
```

终端 B：

```bash
source ~/m20_ros_env.sh
ros2 multicast send
```

如果组播测试失败，`ros2 topic list` 跨主机大概率也会失败。优先回到网络、组播转发服务、Domain ID、`ROS_LOCALHOST_ONLY` 排查。

### 1.7 确认点云类型、QoS 和字段

看到 `/LIDAR/POINTS` 后，继续确认类型：

```bash
ros2 topic type /LIDAR/POINTS
ros2 topic info -v /LIDAR/POINTS
ros2 topic hz /LIDAR/POINTS
ros2 topic echo --once /LIDAR/POINTS
```

如果 `ros2 topic list` 里有 `/LIDAR/POINTS`，但 `ros2 topic hz` 提示 `does not appear to be published yet`，优先尝试 sensor-data QoS：

```bash
ros2 topic hz /LIDAR/POINTS --qos-profile sensor_data
ros2 topic echo --once /LIDAR/POINTS --qos-profile sensor_data
```

如果当前 ROS2 Foxy 的 CLI 不支持 `--qos-profile sensor_data`，改用：

```bash
ros2 topic hz /LIDAR/POINTS --qos-reliability best_effort
ros2 topic echo --once /LIDAR/POINTS --qos-reliability best_effort
```

Lightning-LM 的 RoboSense 预处理分支期望 `PointCloud2` 中有这些字段：

```text
x y z intensity ring timestamp
```

如果没有 `timestamp`，运动畸变补偿会变差，机器人运动时更容易漂。先记录字段，不要急着改代码。

### 1.8 同时确认 IMU

Lightning-LM 还需要 IMU：

```bash
ros2 topic hz /IMU
ros2 topic echo --once /IMU
```

如果 IMU 话题不是 `/IMU`，需要同步修改 Lightning-LM 配置：

```yaml
common:
  imu_topic: "/实际IMU话题名"
```

### 1.9 建议先录一个最小 bag

点云和 IMU 都可见后，先录一个短包，后续可以离线复盘：

```bash
cd ~/robodog_nav_system
source ~/m20_ros_env.sh
mkdir -p bags
ros2 bag record -o bags/m20_lio_smoke /tf /IMU /LIDAR/POINTS
```

录 30 秒即可。机器人先静止几秒，再慢速移动一小段。

## 2. 编译 Lightning-LM

M20 背部主机通常是 Ubuntu 20.04 / ROS2 Foxy。依赖使用 Foxy 版本：

```bash
sudo apt update
sudo apt remove -y libgoogle-glog-dev libgoogle-glog0v5
sudo apt install -y \
  libopencv-dev libpcl-dev pcl-tools libyaml-cpp-dev libepoxy-dev \
  libgflags-dev python3-wheel tmux \
  ros-foxy-pcl-conversions ros-foxy-pcl-ros ros-foxy-tf2-tools
```

不要安装 apt 版 `libgoogle-glog-dev`，它会和仓库自带 glog 冲突。

编译仓库自带 glog：

```bash
cd ~/robodog_nav_system/src/third_party/lightning-lm-deep-robotics/thirdparty/glog
mkdir -p build
cd build
cmake -DBUILD_SHARED_LIBS=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make install
sudo ldconfig
```

编译仓库自带 Pangolin：

```bash
cd ~/robodog_nav_system/src/third_party/lightning-lm-deep-robotics/thirdparty/Pangolin
mkdir -p build
cd build
cmake -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DCMAKE_CXX_FLAGS="-Wno-error" -DCMAKE_BUILD_TYPE=Release ..
make -j2
sudo make install
sudo ldconfig
```

编译 Lightning-LM：

```bash
cd ~/robodog_nav_system
source ~/m20_ros_env.sh
export MAKEFLAGS="-j2"
colcon build --packages-select lightning --parallel-workers 1 --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

如果外接背部主机编译时报 `scrubber_common` 或 `agibot_robot` 找不到，先不要去找 `/opt/robot`。这两个依赖目前只在 `package.xml` 里声明，源码和 CMake 没有实际使用。后续可以在我们的工程副本中移除这两个无用依赖，或把官方接口包一起带到工作空间。

如果是在 AOS/NOS/GOS 内部主机上编译，才需要 source 官方环境脚本。

## 3. 在线 SLAM 建图

运行前确认机器人站稳，现场尽量少动态行人和车辆。

终端 1：

```bash
cd ~/robodog_nav_system
source ~/m20_ros_env.sh
source install/setup.bash

ros2 run lightning run_slam_online \
  --config src/third_party/lightning-lm-deep-robotics/config/default_deep_roboticsslam.yaml
```

终端 2：

```bash
cd ~/robodog_nav_system
source ~/m20_ros_env.sh
source install/setup.bash

ros2 topic echo /lightning/nav_state
```

可选检查：

```bash
ros2 topic hz /lightning/odom
ros2 topic echo /lightning/odom --once
```

保存地图：

```bash
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: 'office4f'}"
```

默认保存路径：

```bash
~/robodog_nav_system/data/office4f/
```

重点文件：

```text
data/office4f/global.pcd
data/office4f/map.pgm
```

## 4. 在线定位

先确认定位配置中的地图路径：

```yaml
system:
  map_path: ./data/office4f/
```

启动定位：

```bash
cd ~/robodog_nav_system
source ~/m20_ros_env.sh
source install/setup.bash

ros2 run lightning run_loc_online \
  --config src/third_party/lightning-lm-deep-robotics/config/default_deep_roboticsloc.yaml
```

检查输出：

```bash
ros2 topic echo /lightning/nav_state
ros2 topic hz /lightning/odom
timeout 3 ros2 topic echo /tf
```

## 5. 实时导航部署路线

离线 bag 只能用于验证 Lightning-LM 或点云处理算法，不能替代最终导航。真正跑导航时，运行导航算法的主机必须实时满足：

```text
能稳定订阅 /LIDAR/POINTS 或等价三维点云
能稳定订阅 /IMU、/ODOM 或 /lightning/odom
能发布 /cmd_vel 或官方导航命令接口
能接入急停、限速、人工接管
```

当前最重要的问题不是“能不能离线建图”，而是“导航算法应该部署在哪台主机上”。

### 5.1 GOS 104 状态

官方资料建议二次开发程序优先部署在 GOS 104，NOS 106 次之，尽量避免占用 AOS/NOS 核心进程资源。因此 GOS 104 是优先考虑的导航算法主机。但当前实测结果显示，GOS 104 还不能直接看到 `/LIDAR/POINTS`。

复查命令：

```bash
ssh user@10.21.31.104
source /opt/robot/scripts/setup_ros2.sh
ros2 topic list | grep -Ei "lidar|point|points|imu|odom"
ros2 topic hz /LIDAR/POINTS
ros2 topic hz /IMU
timeout 3 ros2 topic echo /ODOM
```

当前结果：

```text
GOS 104 能看到 /IMU，约 200Hz。
GOS 104 能看到 /ODOM 话题名。
GOS 104 能看到 /LIDAR/IMU201、/LIDAR/IMU202、/LIDAR/STATUS。
GOS 104 看不到 /LIDAR/POINTS。
```

所以 GOS 104 作为导航主机的前提是先解决实时点云接入。若后续 GOS 104 能稳定看到 `/LIDAR/POINTS`，优先路线才是：

```text
GOS 104:
  Lightning-LM 或其他 LIO/定位
  Nav2 / m20_scan_planner / 三维避障
  安全限速与 /cmd_vel 输出

AOS/NOS:
  保持官方核心感知、通信、底层状态服务
```

这条路线最少折腾外接主机 DDS 跨网段，也更符合官方资源分配建议；但它当前被 `/LIDAR/POINTS` 不可见卡住。

### 5.2 外接背部主机跑导航的条件

如果后续一定要在外接背部主机 `10.21.31.192` 上跑导航算法，则必须先解决实时点云接入，不能只靠 bag。可选方案：

```text
方案 A：让背部主机增加一条到 10.21.33.0/24 的真实网络通路。
方案 B：让 NOS 正确转发 10.21.33 <-> 10.21.31 的 DDS 发现和数据。
方案 C：在 AOS/GOS 上做点云桥接，把 /LIDAR/POINTS 重发布到 10.21.31 可见的 ROS2 话题。
方案 D：导航主算法放在 GOS/AOS，背部主机只做监控、参数、RViz 和日志。
```

对实时导航而言，方案 D 或 GOS 本机部署通常更稳。方案 B/C 涉及高带宽点云跨网段转发，必须重点关注 CPU、网络带宽、丢包和 DDS QoS。

### 5.3 背部主机接入导航前必须通过的实时测试

如果走 Lightning-LM / 三维原始点云建图路线，外接背部主机上至少要通过：

```bash
source ~/m20_ros_env.sh
ros2 topic hz /LIDAR/POINTS
timeout 3 ros2 topic echo /LIDAR/POINTS
ros2 topic hz /IMU
timeout 3 ros2 topic echo /ODOM
```

如果 `/LIDAR/POINTS` 仍不可见，不要在背部主机上启动依赖原始点云的 Lightning-LM 或 SCAN-Planner 原始链路。

如果先走官方处理后点云的导航路线，至少要通过：

```bash
source ~/m20_ros_env.sh
ros2 topic hz /accumulate_cloud/cloud_base
ros2 topic hz /accumulate_cloud/cloud_gravity
ros2 topic hz /passable_area
ros2 topic hz /impassable_area
timeout 3 ros2 topic echo /impassable_area | sed -n '1,120p'
ros2 topic hz /IMU
timeout 3 ros2 topic echo /ODOM
```

这里的重点不是只看话题名，而是确认 `header.frame_id`、频率、延迟、点云字段和高度范围。`/impassable_area` 很可能适合先作为 Nav2 障碍物输入，`/accumulate_cloud/cloud_gravity` 则可能适合三维局部避障验证。

本轮在未发送 `/PASSABLE_AREA_ENABLE` 前，上述处理后点云均没有 `hz` 输出，也没有 `echo` 消息内容。发送一次 `{data: 1}` 后，`/accumulate_cloud/cloud_base`、`/accumulate_cloud/cloud_gravity`、`/passable_area`、`/impassable_area` 和 `/traversal_cost` 已能稳定输出约 1.43Hz 数据，`/passable_status_code=100`。`--no-arr` 已确认它们使用 `base_gravity` 坐标系，其中 `/traversal_cost` 是 8m x 8m、5cm 分辨率的局部 `OccupancyGrid`。RViz 截图已经确认点云可显示。TF 当前约 11Hz，但 `odom` frame 不存在；这条路线已经具备初步可行性，但还没有完成导航验收。下一步必须确认真实 TF 边、`/traversal_cost` 栅格显示、点云/栅格与现场障碍物的一致性、延迟，以及 1.43Hz 频率是否满足 M20 实机速度下的避障安全边界。

## 6. 接入导航前的验收条件

在接 Nav2 或 `m20_scan_planner` 前，至少满足：

```text
/LIDAR/POINTS 或等价可用点云
                     sensor_msgs/msg/PointCloud2，频率稳定
/IMU                  sensor_msgs/msg/Imu，频率稳定
/lightning/nav_state  有连续位姿输出
/lightning/odom       有连续 Odometry 输出
/tf                   至少有 Lightning-LM 发布的定位相关 TF
data/<map_id>/global.pcd 已保存并可读取
```

此时再讨论两条后续路线：

1. 保守路线：`/LIDAR/POINTS -> /scan -> Nav2 + MPPI`。
2. 低速验证路线：`/impassable_area` 或 `/traversal_cost -> Nav2 局部避障候选输入`。
3. 三维路线：`/LIDAR/POINTS + /lightning/odom -> m20_scan_planner`。

不要在点云输入、定位输出和底盘速度限幅未稳定前直接让 M20 执行实物导航。

## 7. 官方部署主机与背部主机分工

### 7.1 官方文档证据

M20 SDK Deploy 的 Sim-to-Real 文档直接把 SDK 包拷到并登录到 AOS 103：

```bash
scp -r ~/sdk_deploy/src user@10.21.31.103:~/sdk_deploy
ssh user@10.21.31.103
cd sdk_deploy
source /opt/ros/foxy/setup.bash
colcon build --packages-select m20_sdk_deploy --cmake-args -DBUILD_PLATFORM=arm
sudo su
source /opt/robot/scripts/setup_ros2.sh
source install/setup.bash
ros2 run m20_sdk_deploy rl_deploy
```

Lightning-LM Deep Robotics 版 README 也明确写到：

```text
M20 Hardware Deployment:
  We test on the AOS(103) platform, which has ROS2_foxy already.

Point Cloud Permissions:
  Enable multicast-relay.service on NOS(106).
  Then switch to AOS(103), enter su mode, source /opt/robot/scripts/setup_ros2.sh,
  and check ros2 topic hz /LIDAR/POINTS.

Recording bags:
  taskset -c 4,5,6,7 chrt 90 ros2 bag record -o lio260310 /tf /IMU /LIDAR/POINTS
```

结合实测结果：

```text
AOS 103 root:
  /LIDAR/POINTS 约 9 - 10Hz
  能看到建图/定位相关话题名，但本轮 /LIO_ODOM 和 /LIO_ODOM_HIGH_FREQUENCY 在 8s 内无输出，
  不能把它们当作已经可用的稳定定位输出

GOS 104:
  能看到 /IMU、/ODOM 和部分 LIDAR 状态话题
  当前看不到 /LIDAR/POINTS

NOS 106:
  正在运行官方 passable_area、accumulate_cloud、localPlanner、localization_dd、pcl_pass_grid 等进程
  背部主机可见的 /traversal_cost、/impassable_area 等低频处理后话题，大概率来自 NOS 上的官方链路

外接背部主机 10.21.31.192:
  能看到 /IMU、/ODOM、/tf、状态话题和官方处理后可通行区域话题
  当前看不到原始 /LIDAR/POINTS
```

所以当前工程判断是：

```text
官方 M20 控制 SDK 部署示例：AOS 103
官方 Lightning-LM / 雷达建图部署测试：AOS 103
一般二次开发更适合的资源主机：GOS 104 或外接背部主机
但依赖原始 /LIDAR/POINTS 的实时算法，目前只能确认 AOS 103 root 可直接拿到输入。
NOS 106 不是空闲转发机，已承载官方导航/可通行区域计算，不建议再作为自研重算法主机。
```

### 7.2 硬件架构图解读

根据硬件架构图，M20 内部不是一个单一扁平网络，而是多个隔离网段通过不同主机和交换机连接。

```text
10.21.33.0/24: 雷达/高带宽传感器网段
  前激光雷达: 10.21.33.201
  后激光雷达: 10.21.33.202
  AOS eth0:   10.21.33.103
  NOS eth0:   10.21.33.106

10.21.31.0/24: 业务/外接/二次开发通信网段
  AOS eth2:   10.21.31.103
  NOS eth1:   10.21.31.106
  GOS eth0:   10.21.31.104
  背部主机:   10.21.31.192
  外部网口:   架构图中的 2/3/4/5 网口

10.21.32.0/24: AOS 与 DCU 的底层控制网段
  AOS eth1:   10.21.32.103
  DCU eth1:   10.21.32.121

10.21.34.0/24: GOS 另一路网口/预留或扩展网段
  GOS eth1:   10.21.34.104

10.21.41.0/24: WiFi / 手柄相关网段
```

图里的“交换机”是内部以太网交换机，主要做二层转发，不等价于会自动跨网段路由。左侧交换机承载雷达网 `10.21.33.x`，右侧交换机承载业务网 `10.21.31.x`。外接背部主机通过机器狗外部网口接入的是右侧业务网，所以拿到的是 `10.21.31.192`，不是雷达网 IP。

DCU 更像底层驱动/电源控制单元，连接关节、电池，并通过 `10.21.32.x` 与 AOS 通信。DCU 不是 ROS 导航算法部署主机，也不应该部署 Nav2、Lightning-LM 或 SCAN-Planner。

三台内部主机的角色可以这样理解：

```text
AOS 103:
  同时接入雷达网、业务网、DCU 控制网。
  是目前唯一确认能直接拿到原始 /LIDAR/POINTS 9 - 10Hz 的主机。
  官方 M20 SDK 和 Lightning-LM 真机说明都把关键部署/验证放在 AOS 103。

NOS 106:
  同时接入雷达网和业务网。
  更像官方导航感知与网络/通信/relay 主机。
  multicast-relay.service active，但当前没有把 /LIDAR/POINTS 完整转发到 GOS 或背部主机。
  本轮进程审计确认它还在运行 passable_area、accumulate_cloud、localPlanner、
  localization_dd、pcl_pass_grid、rslidar、hsLidar 等进程，因此不是空闲计算节点。

GOS 104:
  主要接入业务网 10.21.31.x。
  理论上更适合二次开发，但当前看不到原始 /LIDAR/POINTS。

外接背部主机:
  接入业务网 10.21.31.x。
  适合可视化、日志、调参、上层任务开发。
  默认不能直接访问 10.21.33 雷达网原始点云。
```

这个架构能解释当前实测现象：

```text
1. AOS root 能 hz 到 /LIDAR/POINTS:
   AOS eth0 就在 10.21.33 雷达网，source 官方环境后选择 10.21.33.103。

2. AOS 普通 user 能 list 到 /LIDAR/POINTS，但 hz/echo 可能收不到:
   DDS 发现和实际数据接收不是一回事，官方文档也要求进入 su/root 后检查点云。

3. GOS 104 看不到 /LIDAR/POINTS:
   GOS 在 10.21.31 业务网，未直接接入 10.21.33 雷达网。

4. 背部主机看不到 /LIDAR/POINTS:
   背部主机也在 10.21.31 业务网，不能天然收到 10.21.33 侧的原始高带宽点云。

5. 背部主机能看到 /traversal_cost、/impassable_area 等低频处理后话题:
   这些不是原始 /LIDAR/POINTS 透传，而是官方感知链路处理后的业务网输出。
   进程审计显示 accumulate_cloud 和 passable_area 实际运行在 NOS 106 上。
   当前频率约 1.43Hz，说明它更像可通行区域/地形结果，而不是原始高频避障点云。

6. 背部主机加路由后能 ping 10.21.33.106，但不能 ping 10.21.33.103:
   说明 NOS 自己跨 31/33 两个网，但没有作为完整三层路由器继续转发到整个 10.21.33 网段。
```

因此，如果 GOS 或背部主机要实时使用高频雷达点云，必须走 AOS/NOS 转发或桥接：

```text
方案 1: NOS 做官方 DDS/multicast relay，把 /LIDAR/POINTS 从 10.21.33 转到 10.21.31。
方案 2: AOS 订阅 /LIDAR/POINTS，再发布降采样/ROI 裁剪后的 /obstacle_cloud 或 /LIDAR/POINTS_BRIDGE。
方案 3: AOS 直接运行建图/定位/局部感知，只把低带宽结果发给 GOS/背部主机。
```

当前实测更支持方案 2 或方案 3。方案 1 是否可行取决于官方 relay 配置和 DDS 网络策略，不能只靠外接背部主机加静态路由解决。

### 7.3 当前背部主机适合部署什么

外接背部主机当前适合做：

```text
1. RViz / 可视化 / 远程监控
   /tf、/IMU、/ODOM、/traversal_cost、/impassable_area、/passable_area 已可用于观察。

2. 日志与数据采集
   记录 /IMU、/ODOM、/tf、官方处理后点云、/traversal_cost、状态话题。

3. 任务级状态机和操作界面
   读取 /MOTION_STATE、/MOTION_STATUS、/NAV_STATUS、/PLANNER_STATUS、/FAULT_STATUS、/BATTERY_DATA、/CPU_103/104/106 等。

4. 低速安全监督
   使用 /traversal_cost 或 /impassable_area 做“前方危险则停/减速”的监督层。
   由于频率只有约 1.43Hz，它只能做保守安全辅助，不能作为高速动态避障主闭环。

5. 导航算法离线/半实物开发
   在背部主机上开发 Nav2 参数、costmap 桥接、RViz 配置、bag 回放、地图转换和 UI。
```

外接背部主机当前不适合直接做：

```text
1. 在线 Lightning-LM / FAST-LIO / 原始点云建图定位
   缺少 /LIDAR/POINTS，不能稳定订阅原始 RoboSense 点云。

2. SCAN-Planner 这类依赖实时三维点云的局部规划主闭环
   处理后点云 1.43Hz 太慢，原始点云不可见。

3. 高速 Nav2 + MPPI 动态避障
   MPPI 局部代价图通常需要更高频感知更新。1.43Hz 意味着约 0.7s 更新一次，
   M20 稍微快一点就会在两帧之间移动很远，安全裕度不足。

4. 直接底层运动控制主节点
   SDK/运动控制类官方示例在 AOS 103，背部主机更适合发高层目标或做监督，
   不建议把高频底层控制闭环放在外接链路上。
```

### 7.4 代码部署方案分析与选择

当前推荐分工：

```text
AOS 103:
  保持官方雷达、LIO/建图、底层运动相关链路。
  若要跑 Lightning-LM 在线建图/定位，优先在 AOS 103 root 下验证，并用 taskset 限核。
  若要让 GOS/背部主机使用高频点云，可在 AOS 上做轻量桥接、降采样或 ROI 裁剪。
  本轮资源审计显示 AOS 能拿到原始点云，但已经运行 rslidar、lio_ddsnode、rl_deploy、
  yesense_node、height_map_nav 等官方进程，新增任务必须轻量、限核、可随时关闭。

外接背部主机:
  跑 RViz、日志、bag、UI、任务状态机、Nav2/MPPI 参数实验、低速安全监督。
  先不要承担高频感知闭环。

GOS 104:
  当前资源最空，适合作为后续二次开发和导航主算法主机。
  但当前看不到 /LIDAR/POINTS，所以要等官方 DDS/relay/桥接解决后再承载三维导航主算法。

NOS 106:
  保持官方 passable/localization/localPlanner/网络 relay 等功能。
  不建议部署 Lightning-LM、Nav2、m20_scan_planner 等重计算主算法。
```

可选代码部署方案：

```text
方案 A：AOS 103 跑完整感知/定位/导航
  优点:
    原始点云和官方 LIO 链路最完整，数据路径最短。
  风险:
    AOS 已承担官方核心进程和底层控制相关链路，额外重计算可能抢资源。
  适用条件:
    AOS 资源审计后仍有足够 CPU/内存/温度余量，并且所有新增进程都限核、限优先级。

方案 B：AOS 103 轻量桥接，GOS 104 跑导航主算法
  优点:
    AOS 只做原始点云订阅、降采样、ROI 裁剪或障碍云生成；
    GOS 承载 Nav2/MPPI/m20_scan_planner/任务状态机，更符合二次开发主机定位。
  风险:
    必须打通 AOS -> GOS 的高频点云或障碍物话题，目标至少 8 - 10Hz。
  当前倾向:
    如果 GOS 资源审计结果足够，这是最值得推进的中长期方案。

方案 C：AOS 103 轻量桥接，背部主机跑导航主算法
  优点:
    开发、可视化、调参最方便，算力可能更充足。
  风险:
    外接链路和 DDS 网络更复杂，实机安全依赖外部网线/主机稳定性。
  适用条件:
    背部主机必须稳定拿到 8 - 10Hz 的降采样点云/障碍云，并且有本机或机器狗侧安全兜底。

方案 D：NOS 106 做完整转发或算法主机
  优点:
    NOS 同时接 31/33 两个网段，理论上适合做受控网络转发。
  风险:
    NOS 已经运行官方 localization、localPlanner、pcl_pass_grid、accumulate_cloud、passable_area 等进程，
    负载明显高于 AOS/GOS；当前实测也没有完整转发 /LIDAR/POINTS。
  当前倾向:
    只考虑保持官方 relay/网络配置和官方可通行区域链路，不作为自研导航算法主机。

方案 E：只使用背部主机当前可见的 /traversal_cost 或 /impassable_area
  优点:
    已经能显示和订阅，开发链路最短。
  风险:
    频率约 1.43Hz，不能支撑高速动态避障主闭环。
  适用条件:
    只做低速安全监督、可视化或算法原型验证，不作为最终主感知输入。
```

如果目标是可落地的 M20 实机导航，优先级建议是：

```text
1. 资源审计后的当前首选是“方案 B：AOS 桥接 + GOS 导航主算法”。
2. 优先验证 AOS -> GOS 的轻量高频点云/障碍云桥接，目标 8 - 10Hz。
3. 如果桥接稳定，则 GOS 承担 Nav2/MPPI、m20_scan_planner 或任务状态机。
4. 背部主机保留为 RViz、bag、调参、监控和开发工作站。
5. 如果 GOS 拿不到高频点云，则退回 AOS 限核运行感知/定位，GOS/背部主机只做上层任务与监控。
6. 只有当原始点云或等价障碍物输入达到 8 - 10Hz，并且 TF、定位、底盘速度限幅都稳定后，再接 Nav2 + MPPI 或 m20_scan_planner 主闭环。
```

### 7.5 三台内部主机资源审计

中长期部署前，必须分别审计 AOS 103、GOS 104、NOS 106 的 CPU、内存、磁盘、温度和当前进程占用。不要只凭“推荐二次开发主机”判断。

已知话题可见性：

```text
AOS 103:
  root + source /opt/robot/scripts/setup_ros2.sh 后，/LIDAR/POINTS 可达到约 9 - 10Hz。
  普通 user 下 ros2 topic list 能看到 /LIDAR/POINTS，但 hz/echo 可能收不到实际数据。
  能看到完整官方建图/定位链路话题，如 /LIO_ODOM、/LIO_ODOM_HIGH_FREQUENCY、
  /LOC_BODY_POINTS、/ACCUMULATED_POINTS_MAP、/local_path、/global_path 等。

GOS 104:
  能看到 /IMU 约 200Hz、/ODOM 话题名、/LIDAR/IMU201、/LIDAR/IMU202、/LIDAR/STATUS。
  当前看不到 /LIDAR/POINTS 实际数据。

NOS 106:
  已确认双网卡：eth0=10.21.33.106/24，eth1=10.21.31.106/24。
  multicast-relay.service active。
  已运行官方 passable/localization/localPlanner 等链路，不建议把重计算导航算法放在 NOS。
```

本轮资源审计结果：

```text
外接背部主机 M20piper:
  CPU: Intel N97，4 核，最高约 3.6GHz
  内存: 7.5Gi，总体空闲约 6.8Gi
  磁盘: 根分区 114G，总可用约 83G
  负载: load average 约 0.00 / 0.03 / 0.01
  结论: 资源很空，适合 RViz、日志、bag、UI、开发和离线验证；
        但当前没有原始 /LIDAR/POINTS，不适合直接承担实时三维避障主闭环。

AOS 103:
  CPU: aarch64，8 核 Cortex-A55，最高约 2.3GHz
  内存: 15Gi，总体可用约 12Gi，无 swap
  磁盘: eMMC 约 115G，/var/opt/robot/data 约 62.5G，/userdata 约 18.4G
  负载: load average 约 1.00 / 1.09 / 0.66
  关键进程: rslidar、hsLidar、lio_ddsnode、rl_deploy、yesense_node、height_map_nav 等
  点云: root 下 /LIDAR/POINTS 约 9.1Hz；/LIO_ODOM 与 /LIO_ODOM_HIGH_FREQUENCY 本轮 8s 内无输出
  结论: 是原始点云入口，适合做数据源、bag 录制、轻量桥接或受限的建图验证；
        不建议直接叠加完整导航主闭环，除非限核并持续监控温度/负载。

GOS 104:
  CPU: aarch64，8 核 Cortex-A55，最高约 2.3GHz
  内存: 15Gi，总体可用约 13Gi，无 swap
  磁盘: eMMC 约 115G，根分区占用很低
  负载: load average 约 0.08 / 0.21 / 0.26
  关键进程: rslidar、hsLidar、系统桌面/状态进程，整体很空
  点云: 能看到 /IMU、/ODOM、LIDAR 状态话题，但当前看不到原始 /LIDAR/POINTS
  结论: 当前最适合承载自研导航主算法；前提是先从 AOS 获得 8 - 10Hz 的点云/障碍云桥接。

NOS 106:
  CPU: aarch64，8 核 Cortex-A55，最高约 2.35GHz
  内存: 15Gi，总体可用约 11Gi，无 swap
  磁盘: eMMC 约 115G，根分区占用很低
  负载: load average 约 4.34 / 5.41 / 3.69
  关键进程: pcl_pass_grid、localization_dd、yesense_node、rslidar、localPlanner、
            accumulate_cloud、passable_area、astar_node、hsLidar 等
  已确认进程: /opt/ros/foxy/bin/ros2 launch passable_area nav.launch.py、
              /opt/ros/foxy/lib/accumulate_cloud/accumulate_cloud、
              /opt/ros/foxy/lib/passable_area/passable_area
  结论: NOS 已经是官方通行区域/局部规划/定位相关计算节点，负载最高；
        不适合作为自研 Lightning-LM、Nav2、SCAN/m20_scan_planner 的主机。
```

按这轮资源审计，部署优先级调整为：

```text
首选:
  AOS 103 做原始点云接入与轻量桥接；
  GOS 104 跑导航主算法；
  背部主机做 RViz、日志、调参、开发；
  NOS 106 保持官方链路，不加重。

备选:
  AOS 103 限核运行 Lightning-LM/建图验证，再把定位或障碍物结果发给 GOS/背部主机。

不推荐:
  NOS 106 跑自研重算法；
  背部主机在没有高频桥接点云时直接跑实时三维避障主闭环。
```

分别在三台主机上执行资源审计：

```bash
# AOS
ssh user@10.21.31.103

# GOS
ssh user@10.21.31.104

# NOS
ssh user@10.21.31.106
```

每台主机执行：

```bash
echo "===== HOST ====="
hostname
date
uname -a
cat /etc/os-release | sed -n '1,6p'

echo "===== CPU ====="
lscpu | sed -n '1,35p'
nproc --all
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq 2>/dev/null

echo "===== MEMORY ====="
free -h

echo "===== DISK ====="
df -h /
lsblk -o NAME,SIZE,TYPE,MOUNTPOINT,FSTYPE

echo "===== LOAD/TOP ====="
uptime
ps -eo pid,ppid,comm,%cpu,%mem --sort=-%cpu | head -n 20

echo "===== TEMP ====="
for z in /sys/class/thermal/thermal_zone*/temp; do
  echo "$z: $(cat $z 2>/dev/null)"
done

echo "===== ROS TOPICS SUMMARY ====="
source /opt/robot/scripts/setup_ros2.sh
ros2 topic list | wc -l
ros2 topic list | grep -Ei "LIDAR|POINT|LOC|LIO|IMU|ODOM|path|cost|passable|traversal|NAV|PLANNER"
```

AOS 103 额外执行 root 点云验证：

```bash
su
source /opt/robot/scripts/setup_ros2.sh
timeout 8 ros2 topic hz /LIDAR/POINTS
timeout 8 ros2 topic hz /LIO_ODOM
timeout 8 ros2 topic hz /LIO_ODOM_HIGH_FREQUENCY
```

如果要评估某台主机是否能跑 Lightning-LM 或导航算法，重点看：

```text
CPU:
  大核是否空闲；持续负载是否已经接近满载。

内存:
  空闲内存是否至少有 2 - 4GB，编译/建图还需要更多。

磁盘:
  是否有足够空间存 bag 和地图；单次点云 bag 很容易数 GB。

温度:
  持续高温会降频，影响 LIO 和导航稳定性。

ROS:
  是否能稳定订阅 /LIDAR/POINTS 或等价高频点云；
  如果只能看到低频 /traversal_cost，则不能作为高频避障主输入。
```
