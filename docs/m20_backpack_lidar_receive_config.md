# 山猫 M20 背部主机雷达数据接收配置方案

本文基于 `docs/山猫M20外部主机雷达数据接收配置指南.pdf` 改写，面向真实加装的背部 Ubuntu 主机，而不是 VMware 虚拟机。核心思路不变：背部主机通过机器狗外部网口接入 `10.21.31.0/24` 业务网，由机器人侧 `multicast-relay.service` 将前后 RoboSense 雷达组播数据转发到业务网，背部主机运行 `rslidar_sdk` 解码并发布 ROS2 `PointCloud2` 话题。

## 0. 本次实机结论

当前链路已经验证到前雷达点云发布成功：

```text
NOS eth1 -> 背部主机 enp2s0:
224.10.10.201:6691  前雷达 MSOP，高频点云主数据
224.10.10.202:6692  后雷达 MSOP，高频点云主数据
224.10.10.201:7781  前雷达 DIFOP，低频配置/状态数据
224.10.10.202:7782  后雷达 DIFOP，低频配置/状态数据

ros2 topic hz /rslidar_points_front:
average rate: 10.00 Hz
```

这次“突然可以了”的直接原因是：NOS 上的 `multicast-relay.service` 原本处于半正常状态。服务显示 `active (running)`，但只有 `Tasks: 3`，`python3` 只持有 `7781/7782`，没有持有 `6691/6692`，所以背部主机只能收到 DIFOP，收不到 MSOP，SDK 报 `ERRCODE_MSOPTIMEOUT`。

执行下面命令后，服务重新创建四个转发线程：

```bash
sudo systemctl restart multicast-relay.service
```

重启后状态变为 `Tasks: 5`，`python3` 同时持有 `6691/6692/7781/7782`，NOS `eth1` 和背部主机 `enp2s0` 都能抓到高频 `6691/6692`，因此 rslidar SDK 开始发布点云。

因此后续判断不能只看 `Active: active (running)`。健康标准应同时满足：

- `systemctl status multicast-relay.service` 显示 `Tasks: 5`。
- `ss -ulpn` 中 `python3` 持有 `224.10.10.201:6691`、`224.10.10.202:6692`、`224.10.10.201:7781`、`224.10.10.202:7782`。
- NOS `eth1` 能抓到 `6691/6692`。
- 背部主机 `enp2s0` 能抓到 `6691/6692/7781/7782`。
- `/rslidar_points_front` 和 `/rslidar_points_rear` 有稳定频率。

后续复测发现该问题会复发：长时间停用、整机重启、网络接口重新初始化或官方 `rslidar` 进程重启后，`multicast-relay.service` 可能仍显示 `active`，但 `6691/6692` 两个 MSOP 高频转发线程没有实际工作。此时重新执行 `restart` 会重新绑定端口、重新加入组播组并重新创建转发线程，所以背部主机立刻恢复收到 `6691/6692`。因此当前应把 NOS 上的 relay 健康检查作为每次启动 `rslidar_sdk` 前的固定预检步骤。

目前已经确认 `/rslidar_points_front` 为 `10 Hz`，后续还需确认 `/rslidar_points_rear`、点云字段、RViz 显示、rosbag 录制和重启后的持久性。

### 0.1 官方自带雷达/感知话题分层记录

最新在 NOS root 下补充验证后，可以把 M20 自带雷达相关话题分成三层看：

| 层级 | 典型话题/端口 | 当前理解 |
| --- | --- | --- |
| 原始 UDP 包 | `224.10.10.201:6691/7781`、`224.10.10.202:6692/7782` | 两颗 RoboSense 雷达的原始组播包。`6691/6692` 是 MSOP 点云主数据，`7781/7782` 是 DIFOP 设备/配置/状态数据。NOS 的 `multicast-relay.service` 只转发 UDP，不做点云拼接。 |
| 官方原始点云 DDS/ROS2 话题 | `/LIDAR/POINTS` | NOS root 下 `ros2 topic info -v /LIDAR/POINTS` 显示 `Publisher count: 2`，发布者为 `_CREATED_BY_BARE_DDS_APP_`。这说明它不是普通 ROS2 节点发布，而是底层 DDS 应用发布；更像两个发布端共用同一个 topic。 |
| 官方状态/辅助话题 | `/LIDAR/IMU201`、`/LIDAR/IMU202`、`/LIDAR/STATUS` | 更偏雷达 IMU、状态、健康信息。外接背部主机或 GOS 有时能发现这些话题，但这不等价于已经能收到高带宽 `/LIDAR/POINTS`。 |
| 官方处理后局部感知结果 | `/accumulate_cloud/cloud_base`、`/accumulate_cloud/cloud_gravity`、`/passable_area`、`/impassable_area`、`/traversal_cost` | 由 NOS 上 `accumulate_cloud`、`passable_area` 等官方链路处理后输出，通常已经经过累计、重力对齐、可通行/不可通行区域分割。实机使能 `/PASSABLE_AREA_ENABLE` 后可输出约 `1.43 Hz`，可作为低速导航/监督层候选，但不是原始双雷达点云。 |

其中 `/LIDAR/POINTS` 最容易误解。最新日志显示：

```text
Type: sensor_msgs/msg/PointCloud2
Publisher count: 2
Node name: _CREATED_BY_BARE_DDS_APP_
Node name: _CREATED_BY_BARE_DDS_APP_
```

同时连续 `echo --no-arr` 看到的消息均为 `frame_id: lidar_link`，但点数并不固定：

```text
width: 45808
width: 45657
...
width: 17626
width: 68315
```

特别是后两帧时间戳非常接近，点数差异却很大。因此当前最可信判断是：

```text
前雷达 UDP 224.10.10.201:6691/7781 \
                                       -> 官方 rslidar/bare DDS -> /LIDAR/POINTS
后雷达 UDP 224.10.10.202:6692/7782 /
```

也就是说，`/LIDAR/POINTS` 很可能是两个底层 DDS publisher 共用一个 ROS2 topic，订阅者会收到交错到达的点云消息流。
ROS2/DDS 允许多个 publisher 使用同一个 topic，只要消息类型一致即可；但这种共用 topic 不等价于 ROS2 自动把前后雷达拼接成一帧融合点云。

是否已经在官方 `rslidar` 内部做了外参变换或局部合并，还需要继续通过连续帧 `width`、`stamp`、`fields`、点云空间分布和 rosbag 分析确认。当前不能简单把 `/LIDAR/POINTS` 当成“已拼接好的双雷达全景点云”，也不能把 `/accumulate_cloud`、`/impassable_area` 当成原始点云。

### 0.2 多主机时间同步风险

本次实机还发现：背部主机、AOS、GOS、NOS 并不处在同一个标准时间上。按 `date '+%s.%N %F %T'` 的前后夹测估算，AOS 比背部主机快约 `4min49s`，GOS/NOS 比背部主机快约 `8min32s`，且 GOS 与 NOS 更接近，AOS 与 GOS/NOS 又相差约 `3min43s`。

最新时间服务检查显示：

| 主机 | 时间服务现象 | 当前理解 |
| --- | --- | --- |
| 背部主机 | 最初为 `systemd-timesyncd` 客户端；本轮为给 NOS 授时，已安装 `chrony` 并对 `10.21.31.0/24` 开放 NTP | 当前作为外部开发基准；短期可作为局域网时间源，长期还要确认其自身上游 NTP/PTP 是否稳定。 |
| AOS 103 | `chrony` active，日志选中过外部 NTP；同时有 `ptp4l -i eth0 -s`、`phc2sys -s eth0 -c CLOCK_REALTIME`、`gps_node` | 同时存在 NTP/PTP/GPS 相关链路，最终时间源需要继续确认。 |
| GOS 104 | `chrony` active，`ptp4l -i eth0 -s`，`phc2sys -s eth0 -c CLOCK_REALTIME` | 更像 PTP slave，通过网卡 PHC 校系统时间，所以与 NOS 很接近。 |
| NOS 106 | `chrony` active，但 `timedatectl` 显示 `System clock synchronized: no`；`ptp4l` 跑在 `eth0/eth1`，`phc2sys -s CLOCK_REALTIME -c eth0/eth1` | 更像把 NOS 系统时间写到两张网卡 PHC，再服务 33/31 两个网段；但 NOS 自身没有确认锁定到标准时间。 |

后续 `chronyc tracking/sources` 和 `pmc` 复测进一步坐实了这个判断：

- AOS 的 `chronyc sources -v` 有 8 个外部 NTP 源，当前选中 `alphyn.canonical.com`，但 `tracking` 仍显示 `System time: 347.655s fast of NTP time`，说明系统时间还没有被拉回标准时间，或者正在被 `chrony` 与 PTP/PHC 链路共同影响。
- GOS 的 `chronyc sources -v` 为 `Number of sources = 0`，`Leap status: Not synchronised`，所以它的 `chrony active` 只是服务在运行，不代表已经同步到 NTP。
- NOS 的 `chronyc sources -v` 同样为 `Number of sources = 0`，`Leap status: Not synchronised`，但它同时运行 `ptp4l` 和 `phc2sys -s CLOCK_REALTIME -c eth0/eth1`，所以更像 PTP 时间的源头，而不是 NTP 客户端。
- AOS 的 `pmc GET TIME_STATUS_NP` 显示 `gmPresent: true`，`gmIdentity: 56f1d2.fffe.6604fd`；GOS 显示 `gmPresent: true`，`gmIdentity: 5af1d2.fffe.6604fd`；这两个 ID 与 NOS 两张网卡的 MAC 派生形式高度一致。
- NOS 的 `pmc` 显示 `gmPresent: false`，`gmIdentity: 5af1d2.fffe.6604fd`，这通常表示查询到的 PTP 实例本身就是 grandmaster。

因此当前更准确的链路是：NOS 在 33 网段和 31 网段分别通过 PTP/PHC 向 AOS、GOS 等主机提供时间；GOS 基本跟着 NOS；AOS 一边跟着 NOS 的 PTP，一边又能访问外部 NTP/GPS 链路，所以出现了更复杂的偏差状态。真正没有被确认同步到标准时间的是 NOS，而 NOS 又是内部 PTP 时间源，这就是内部时间整体偏离背部主机的根因。

这里容易误判 GOS。GOS 的 `chronyc sources = 0`、`Leap status: Not synchronised` 不代表它脱离了 NOS，因为 GOS 的系统时间不是靠 NTP/chrony 校准，而是靠 `ptp4l + phc2sys`。本轮 GOS 的 `pmc` 显示 `gmPresent: true`、`gmIdentity: 5af1d2.fffe.6604fd`、`master_offset: 285ns`，从 PTP 角度看它实际正在跟随 NOS 31 网段上的 grandmaster。若 `date` 上看起来不一致，应优先确认 `phc2sys.service` 是否仍 active、PTP 偏差是否仍为纳秒/微秒级、以及 NOS 时间是否真的已经被成功修改。

这里要特别看 `phc2sys` 的方向：

- `phc2sys -s eth0 -c CLOCK_REALTIME`：用网卡硬件时钟 PHC 校系统时间。
- `phc2sys -s CLOCK_REALTIME -c eth0/eth1`：用系统时间校网卡硬件时钟 PHC。

因此，“NTP service active” 不等于“时间已经和标准时间一致”。后续如果只在背部主机上解码雷达并做局部可视化，可以临时把 `rslidar_sdk` 的 `use_lidar_clock` 改为 `false`，让新发布的 `/rslidar_points_front`、`/rslidar_points_rear` 使用背部主机系统时间戳。但只要要融合机器狗内部的 `/odom`、`/tf`、`/LIDAR/POINTS`、IMU 或官方状态话题，就应该先统一四台机器的时钟，否则很容易出现 TF extrapolation、message filter 丢数据、costmap 认为数据来自未来或过期等问题。

正式跑导航前建议补做：

```bash
chronyc tracking
chronyc sources -v
chronyc sourcestats -v
systemctl list-units --type=service --all | grep -Ei 'ptp|phc|chrony|gps'
systemctl cat 'ptp*' 'phc*' chrony 2>/dev/null
sudo pmc -u -b 0 'GET TIME_STATUS_NP'
```

工程上建议最终只选一个权威时间源：要么让背部主机作为局域网 NTP/chrony server，同步 AOS/GOS/NOS；要么沿用官方 PTP/NTP 架构，但先确认 NOS/AOS/GOS 的主从关系和最终时间源，不要让 chrony、PTP、GPS 同时互相抢系统时间。

若按官方最小方案临时校准 NOS，远程执行 `sudo date -s` 需要分配交互式 TTY。下面这个写法会失败：

```bash
ssh user@10.21.31.106 "sudo date -s @$TS"
```

失败日志为 `sudo: a terminal is required to read the password`。应改成：

```bash
TS=$(date +%s)
ssh -t user@10.21.31.106 "sudo date -s @$TS && sudo hwclock -w || true"
```

或者先 `ssh user@10.21.31.106` 登录 NOS，再在交互终端里手动执行 `sudo date -s ...` 和 `sudo hwclock -w`。

本次实机已验证下面的短期校准命令可以真正执行到 NOS 上：

```bash
TS=$(date +%s)
ssh -tt user@10.21.31.106 "sudo date -s @$TS && (sudo hwclock -w || true)"
```

执行时会依次输入 SSH 密码和 NOS 上 `sudo` 密码，成功日志形如：

```text
Tue 21 Jul 2026 10:42:01 AM CST
Connection to 10.21.31.106 closed.
```

这表示 NOS 系统时间已经被设置为背部主机执行 `TS=$(date +%s)` 时刻的时间。由于日志中尚未补充设置后的四机夹测结果，当前只能确认“写 NOS 时间成功”，还需继续确认 AOS/GOS 是否通过 PTP 跟随到新时间域。

注意：该短期命令中的 `TS=$(date +%s)` 在输入 SSH 密码和 `sudo` 密码之前执行，因此最终精度会包含人工输密码和建连耗时。它适合把数分钟级偏差快速拉回，但不保证亚秒级同步。导航前必须用夹测命令确认最终偏差；若后续要求稳定小于 0.5s 或几十毫秒，应切换到长期 PTP/NTP 自动同步方案。

本轮 30 秒后复测结果为：

```text
AOS 103     1784602377.626425193 2026-07-21 10:52:57 CST
GOS 104     1784602379.912838102 2026-07-21 10:52:59 CST
NOS 106     1784601872.637542042 2026-07-21 10:44:32 CST
localhost   1784601878.878972391 2026-07-21 10:44:38 CST
```

结论：

- NOS 已经从原先约 `+8min32s` 的偏差被拉回到距离背部主机约 `6.24s`，说明写 NOS 时间成功。
- 这 `6.24s` 基本就是 `TS=$(date +%s)` 之后输入 SSH 密码和 `sudo` 密码产生的延迟。
- AOS/GOS 仍保持旧时间，说明在不重启或不重启 PTP/PHC 服务的情况下，它们没有立即跟随 NOS 的阶跃时间修改；这与官方手册“完成更改后重启山猫”的要求一致。

随后用 SSH 复用连接只做了夹测，结果仍然稳定显示 NOS 慢于背部主机约 `6.234s`：

```text
check 1: local_mid=1784602208.222328630, nos=1784602201.987828208, offset=-6.234500422s
check 2: local_mid=1784602208.242818148, nos=1784602202.009158992, offset=-6.233659156s
check 3: local_mid=1784602208.263825759, nos=1784602202.030057227, offset=-6.233768532s
```

这一步只是验证，没有重新执行 `sudo date -s`，所以偏差没有被进一步消除。

若只想把 NOS 与背部主机对得更准，可先建立 SSH 复用连接，避免校时时再次输入 SSH 密码；再用 `sudo -S` 避免交互式 sudo 延迟。该方法仍是短期手动校准，不是长期自动同步：

```bash
# 1. 建立到 NOS 的 SSH 复用连接；这里会输入一次 SSH 密码
ssh -MNf \
  -o ControlMaster=yes \
  -o ControlPath=/tmp/m20_nos_ctl \
  -o ControlPersist=10m \
  user@10.21.31.106

# 2. 本地读取 NOS sudo 密码，不回显；随后立即取高精度时间并写入 NOS
read -rsp 'NOS sudo password: ' NOS_SUDO_PASS; echo
TS=$(date '+%s.%N')
printf '%s\n' "$NOS_SUDO_PASS" | ssh -S /tmp/m20_nos_ctl user@10.21.31.106 \
  "sudo -S sh -c 'date -s @$TS; hwclock -w 2>/dev/null || true; date +%s.%N'"
unset NOS_SUDO_PASS

# 3. 验证 NOS 是否落在本地前后夹测时间之间
for i in 1 2 3; do
  echo "===== check $i ====="
  echo "local_before $(date '+%s.%N %F %T')"
  ssh -S /tmp/m20_nos_ctl user@10.21.31.106 "echo nos \$(date '+%s.%N %F %T')"
  echo "local_after  $(date '+%s.%N %F %T')"
done

# 4. 用完后可关闭复用连接
ssh -O exit -S /tmp/m20_nos_ctl user@10.21.31.106
```

验证时，如果 NOS 的 epoch 基本落在 `local_before` 与 `local_after` 之间，说明 NOS 与背部主机已经在本次短期调试所需范围内对齐。严格意义上的“完全同步”需要持续 NTP/PTP，手动 `date -s` 只能保证设置瞬间接近，之后仍会随本机晶振缓慢漂移。

也可以让 NOS 直接通过时间同步协议去同步背部主机，这不一定等同于长期方案。区别在于：

- 一次性同步：临时启动或调用 NTP/chrony 客户端，同步完成后退出，属于短期调试方案。
- 持续同步：配置 systemd 服务、chrony 配置或 PTP Boundary Clock，开机自动运行并持续修正漂移，才属于长期部署方案。

若要做一次性 NTP 同步，背部主机必须先提供 NTP 服务；`systemd-timesyncd` 只能做客户端，不能给 NOS 授时。推荐用 `chrony` 在背部主机上临时/长期提供 LAN NTP：

```bash
# 背部主机：安装并允许 10.21.31.0/24 访问本机 NTP
sudo apt update
sudo apt install -y chrony
sudo cp /etc/chrony/chrony.conf /etc/chrony/chrony.conf.bak.$(date +%s)
printf '\n# M20 LAN time server\nallow 10.21.31.0/24\nlocal stratum 10\n' | \
  sudo tee -a /etc/chrony/chrony.conf
sudo systemctl restart chrony
chronyc tracking
sudo ss -ulpn | grep ':123'
```

然后在 NOS 上做一次性同步。由于 NOS 上已有 `chrony.service`，且当前没有 NTP sources，可短暂停掉它，使用 `chronyd -q` 同步一次后再恢复：

```bash
ssh user@10.21.31.106
sudo systemctl stop chrony
sudo chronyd -q 'server 10.21.31.192 iburst'
sudo hwclock -w || true
sudo systemctl start chrony
date '+%s.%N %F %T %Z %z'
```

本轮实机执行到这一步后，背部主机已经能作为 NTP 服务端监听 `UDP/123`：

```text
Reference ID    : 7F7F0101 ()
Stratum         : 10
Leap status     : Normal

UNCONN 0 0 0.0.0.0:123 0.0.0.0:* users:(("chronyd",pid=43271,fd=7))
```

这里的 `Reference ID: 7F7F0101` 和 `Stratum: 10` 表示背部主机当前在用 `local stratum 10` 把自己的本地时钟作为局域网授时源。短期调试是可以接受的，因为目标是让 NOS 与背部主机对齐；但它不等于“背部主机已经锁定外网 NTP”。长期运行前还应在背部主机补查：

```bash
chronyc sources -v
chronyc tracking
```

NOS 上的一次性同步已经成功，关键日志为：

```text
2026-07-21T02:56:56Z System clock wrong by 6.246399 seconds (step)
2026-07-21T02:57:02Z chronyd exiting
```

这表示 `chronyd -q` 已经从 `10.21.31.192` 获取时间，并把 NOS 系统时钟阶跃修正了 `6.246399s`。这个数值与前面手动 `date -s` 后遗留的约 `6.234s` 偏差吻合，因此可以判断：NOS 已经完成了一次协议级校时，比继续手输 `date -s` 更准。

重启山猫之前，先在背部主机做一次 NOS 与背部主机的夹测，确认残差：

```bash
for i in 1 2 3 4 5; do
  lb=$(date +%s.%N)
  rt=$(ssh -S "$CTL" user@10.21.31.106 "date +%s.%N")
  la=$(date +%s.%N)
  awk -v i="$i" -v lb="$lb" -v rt="$rt" -v la="$la" \
    'BEGIN { mid=(lb+la)/2; printf("check %d: offset=%+.6fs rtt=%.6fs nos=%.9f local_mid=%.9f\n", i, rt-mid, la-lb, rt, mid) }'
done
```

判据：若 `nos` 的 epoch 落在 `local_before` 与 `local_after` 之间，或相对本地中点偏差小于约 `0.5s`，就可以认为短期导航复现需要的 NOS/背部主机时间已经对齐。随后再按官方手册重启整机，让 AOS/GOS/雷达通过内部 PTP/PHC 链路重新跟随 NOS。

本轮实机夹测已经通过，结果为：

```text
check 1: offset=-0.001683s rtt=0.018566s
check 2: offset=-0.001774s rtt=0.017724s
check 3: offset=-0.000966s rtt=0.018623s
check 4: offset=-0.001506s rtt=0.016970s
check 5: offset=-0.001506s rtt=0.016908s
```

这说明 NOS 当前约比背部主机慢 `1-2ms`，SSH 往返耗时约 `17-19ms`。该结果已经远好于短期导航复现的 `0.5s` 目标，可以进入重启整机后的 AOS/GOS/NOS/背部主机四机复测。

整机重启后，三台内部主机已能重新 SSH，说明 AOS/GOS/NOS 网络和系统均已恢复。顺序输密码粗测结果为：

```text
AOS 103     1784608005.572051498 2026-07-21 12:26:45 CST
GOS 104     1784608006.929101980 2026-07-21 12:26:46 CST
NOS 106     1784608008.337164692 2026-07-21 12:26:48 CST
localhost   1784608006.482401524 2026-07-21 12:26:46 CST
```

这次不再出现此前 `4-8min` 级偏差，说明重启后没有明显回退到旧时间域；但该命令是串行 SSH 并包含人工输密码，不能用来判断毫秒级误差。重启后的严格判断仍需对 AOS/GOS/NOS 分别做 `local_before / remote / local_after` 夹测，或建立 SSH 复用后计算 offset。

继续做三台主机的夹测时，直接看到的 `offset` 分别为 `+2s` 到 `+6s`，但这不是三台主机各自乱漂。原因是每次 SSH 都重新输入密码，`date` 实际在本地 `local_after` 附近才执行，因此应近似扣除 `rtt/2` 再看残差。修正后结果非常一致：

```text
AOS 103: corrected offset ~= +1.8553s
GOS 104: corrected offset ~= +1.8554s
NOS 106: corrected offset ~= +1.8554s
```

因此更准确的结论是：重启后 AOS/GOS/NOS 三台内部主机仍然彼此同步，内部 PTP 时间域没有散；但整个内部时间域相对背部主机约快 `1.855s`，没有保持到重启前的 `1-2ms` 状态。若继续做导航复现，建议再对 NOS 执行一次 `sudo chronyd -q 'server 10.21.31.192 iburst'`，随后不要立刻重启，等待 `10-30s` 后复测 AOS/GOS/NOS 与背部主机的 offset。

随后按该建议对 NOS 再做一次 `chronyd -q`，日志为：

```text
2026-07-21T05:06:05Z System clock wrong by -1.851979 seconds (step)
```

这里的负号表示 NOS 原本比背部主机快，`chronyd` 将其向后校正了约 `1.852s`。复测时 SSH 复用已生效，`rtt` 回到 `15-18ms`，NOS 相对背部主机只剩约 `+4ms`：

```text
check 1: offset=+0.004844s rtt=0.018203s
check 2: offset=+0.004210s rtt=0.016268s
check 3: offset=+0.004122s rtt=0.014942s
check 4: offset=+0.004560s rtt=0.016041s
check 5: offset=+0.004238s rtt=0.016472s
```

到这一步可确认：重启后的 NOS 已重新与背部主机对齐到毫秒级。短期导航复现可以继续，但若要融合 AOS/GOS 侧话题，仍建议再等 `10-30s` 后对 AOS/GOS 也做一次 SSH 复用夹测，确认内部 PTP 已跟随新的 NOS 时间。

随后对 AOS/GOS 做 SSH 复用夹测，结果为：

```text
AOS 103:
check 1: offset=+0.004058s rtt=0.019806s
check 2: offset=+0.003769s rtt=0.017838s
check 3: offset=+0.003551s rtt=0.016582s
check 4: offset=+0.004274s rtt=0.019536s
check 5: offset=+0.003662s rtt=0.016188s

GOS 104:
check 1: offset=+0.003492s rtt=0.015126s
check 2: offset=+0.003203s rtt=0.013791s
check 3: offset=+0.003046s rtt=0.012812s
check 4: offset=+0.002961s rtt=0.012863s
check 5: offset=+0.002864s rtt=0.012266s
```

这说明 AOS/GOS 已经通过内部 PTP/PHC 链路跟随新的 NOS 时间，三台内部主机相对背部主机均已达到毫秒级。此时不需要再次重启；再次重启反而可能因为长期上游时间源尚未固化而回到启动后的秒级偏差。只有修改了 PTP/chrony/systemd 配置、验证冷启动持久性，或内部主机没有跟随 NOS 时，才需要再按整机重启流程验证。

Lightning 建图重新运行后，日志中仍出现过一次：

```text
E imu_processing.hpp:223] get abnormal dt: 0.508645
E laser_mapping.cc:377] lidar loop back, clear buffer
E laser_mapping.cc:379] lidar loop back, dt: -2.59876e-05
W laser_mapping.cc:185] sync package failed
```

结合源码判断，这已经不是此前多主机时钟相差分钟级导致的严重错位。`get abnormal dt` 的触发条件是 IMU 积分步长 `dt > 0.1s`，本次 `0.508645s` 表示启动期或缓存中存在约半秒 IMU 间隔；`lidar loop back` 的 `dt=-2.6e-05s` 只有约 `26us`，更像点云时间戳近似重复/微小乱序。算法随后继续处理多帧点云并正常保存地图，说明不是致命崩溃。

但这也不是理想状态。若这些错误在建图运行全过程持续刷屏，会影响 LIO 去畸变和 IMU 预积分，导致轨迹抖动、局部地图重影或定位漂移。当前建议：时间同步完成后重启 Lightning/tmux 会话，丢弃启动最初几秒数据，连续运行 `1-2min` 观察 `get abnormal dt`、`lidar loop back`、`sync package failed` 是否仍频繁出现。若只是启动初期偶发，可继续建图；若持续出现，应继续检查 `/LIDAR/POINTS` 与 `/IMU` 的 `header.stamp` 单调性、频率和二者时间差。

若三类日志持续反复出现，则应按输入流问题处理，而不是继续归因于系统时钟。当前最可疑的两个方向：

1. `/LIDAR/POINTS` 是否为双雷达/多发布源共用话题，导致点云帧时间戳微小回退。Lightning 的 `LaserMapping::ProcessPointCloud2()` 对单一 LiDAR 流做了严格单调假设，只要 `msg->header.stamp` 小于上一帧就会触发 `lidar loop back`。
2. `/IMU` 是否存在消息断流、低频或回调队列积压。Lightning 在线模式使用 `rclcpp::QoS(10)`，LiDAR 回调内直接执行较重的 `ProcessLidar`，若 IMU 高频消息在同一 executor 中排队/丢包，就可能让 IMU 预积分步长超过 `0.1s`，触发 `get abnormal dt`。

下一步应先在运行 Lightning 的同一台主机、同一套环境下检查：

```bash
ros2 topic info -v /LIDAR/POINTS
ros2 topic info -v /IMU
timeout 15 ros2 topic hz /LIDAR/POINTS
timeout 15 ros2 topic hz /IMU
```

重点看 `/LIDAR/POINTS` 是否有多个 publisher，以及 `/IMU` 频率是否稳定。若 `/LIDAR/POINTS` 有多个发布源或时间戳不单调，后续需要改成单调的融合点云话题，或临时只取单个雷达源验证；若 `/IMU` 间隔确实反复超过 `0.1s`，则要考虑增大 QoS 队列、使用 sensor-data QoS/多线程 executor，或换用更稳定的 IMU 话题。

本轮 60 秒时间戳探针已经坐实问题集中在 `/LIDAR/POINTS`，而不是 `/IMU`：

```text
/IMU:
  n=12002, back=0, gap=0, maxdt=0.005296s

/LIDAR/POINTS:
  n=600, back=46, gap=62, maxdt=0.200046s
```

同时 `ros2 topic hz` 显示 `/LIDAR/POINTS` 到达频率约 `10Hz`，`/IMU` 约 `200Hz`。这并不矛盾：`ros2 topic hz` 看的是消息到达间隔，而 Lightning 看的是 `header.stamp`。当前 `/LIDAR/POINTS` 的消息到达节奏稳定，但 `header.stamp` 存在微小回退（约 `20-50us`）和约 `0.2s` 的时间戳跳变。结合 `Publisher count: 2`，当前最可信结论是：官方 `/LIDAR/POINTS` 不是一条满足严格单调假设的单 LiDAR 流，不适合直接喂给 Lightning 的 LIO 前端。

这与官方示例视频不一定矛盾。官方 README 中的主要复现实例首先使用 M20 数据包，`ros2 bag play` 回放时对外只有 bag player 一个发布端；而当前真机在线环境里 `/LIDAR/POINTS` 由两个 bare DDS publisher 同时发布。DDS/ROS2 只保证单个 publisher 内部的发送顺序，不保证两个 publisher 的消息按 `header.stamp` 全局排序。即便前后雷达已经被 PTP 同步到同一时间域，两个雷达帧时间仍可能只有几十微秒差异，网络调度和 DDS 投递顺序稍有交错，就会出现“较新的帧先到、较旧的帧后到”。Lightning 的源码在 `LaserMapping::ProcessPointCloud2()` 中直接用 `timestamp < last_timestamp_lidar_` 判定回环，因此这种微小乱序会被当成 `lidar loop back`。

另外，当前 echo 中也能看到 `/LIDAR/POINTS` 的点数并不恒定，例如连续帧出现过 `width=45808`、`45657`、`17626`、`68315`。这更像多个底层来源或不同阶段处理结果共用一个话题，而不是一个已经稳定排序、单消息输出的融合点云。官方视频可能使用的是离线包、单发布端数据、不同固件/配置，或者只是没有展示启动期/运行期日志，因此不能直接据此认定当前输入流一定正常。

下一步不要再优先调系统时钟或 IMU，而应优先验证单源/单调点云输入：

```bash
# 候选 1：单 publisher 的官方点云
ros2 topic info -v /LIDAR/POINTS2
timeout 15 ros2 topic hz /LIDAR/POINTS2

# 候选 2：官方处理后的单 publisher 点云，若当前场景有发布
ros2 topic info -v /ALIGNED_POINTS
ros2 topic info -v /LOC_BODY_POINTS
ros2 topic info -v /NAV_POINTS
```

若候选话题频率和 `header.stamp` 单调性合格，再把 Lightning 配置中的 `common.lidar_topic` 从 `/LIDAR/POINTS` 改到该话题测试。若没有合格候选，则需要新建一个中间节点：短期可以把 `/LIDAR/POINTS` 重发布为单调话题、丢弃 `stamp <= last_stamp` 的帧来验证 Lightning 是否停止 `lidar loop back`；长期应使用单雷达原始流或真正融合后的单调点云流，而不是让两个 bare DDS publisher 交错进入同一个 LIO 前端。

注意：`chronyd -q` 校的是 NOS 系统时钟。若准备立刻重启整机，建议夹测通过后在 NOS 上补执行 `sudo hwclock -w || true`，与官方手册保持一致，尽量避免重启后从旧 RTC/硬件时钟恢复。

这种 `chronyd -q` 是“直接同步外部时钟”的短期方式，精度通常明显好于手动 `date -s`，但它不是长期持续同步。若后续要长期运行，应把 NOS 的上游时间源正式配置为背部主机/外部时钟源，并结合官方 Pro 手册把 NOS 配成 Boundary Clock，上游跟随外部时间，下游继续给 AOS/GOS/雷达授时。

短期复现/调试推荐流程：

```bash
# 1. 背部主机当前时间作为临时标准时间
date '+%s %F %T %Z %z'

# 2. 将背部主机当前 epoch 写入 NOS
TS=$(date +%s)
ssh -tt user@10.21.31.106 "sudo date -s @$TS && (sudo hwclock -w || true)"

# 3. 等待内部 PTP/phc2sys 链路传播
sleep 30

# 4. 夹测 AOS/GOS/NOS/背部主机时间差，避免 SSH 输密码耗时误判
for h in 10.21.31.103 10.21.31.104 10.21.31.106; do
  echo "===== $h ====="
  echo "local_before $(date '+%s.%N %F %T')"
  ssh user@$h "echo remote \$(date '+%s.%N %F %T')"
  echo "local_after  $(date '+%s.%N %F %T')"
done
echo "===== localhost ====="
date '+%s.%N %F %T %Z %z'
```

判定标准：

- NOS 与背部主机的 epoch 差应接近 SSH 往返耗时，通常应小于几秒。
- GOS/AOS 若 PTP 正常，应在数十秒内跟随 NOS；若没有跟随，继续查 `ptp4l.service`、`phc2sys.service` 和 `pmc GET TIME_STATUS_NP`。
- 若只做短期导航复现，校准一次后即可继续雷达、TF、导航验证；若重启后再次偏移，再重复该短期流程。

长期方案先按官方手册记录为待实施方向：让背部主机或外部时间服务器作为 PTP Grandmaster/NTP 源，NOS 作为 Boundary Clock，上游跟随外部时间，下游继续给 AOS/GOS/雷达授时。长期方案的核心目标是开机后自动保持：

```text
背部主机/外部时钟源
  -> NOS 上游接口
  -> NOS Boundary Clock
  -> AOS/GOS/前后雷达/内部传感器
```

长期方案落地前应先确认现场是否允许修改 NOS 的 `ptp4l/phc2sys` 服务配置。未确认前，不建议直接改 `/lib/systemd/system/ptp4l*.service` 或 `/lib/systemd/system/phc2sys*.service`。

## 1. 目标链路

推荐链路如下：

```text
前/后 RoboSense 雷达
  -> M20 内部 10.21.33.x 雷达网
  -> NOS(10.21.31.106 / 10.21.33.106) 官方 multicast-relay.service
  -> M20 外部业务网 10.21.31.0/24
  -> 背部主机有线网口，例如 10.21.31.192
  -> rslidar_sdk
  -> /rslidar_points_front, /rslidar_points_rear 或自定义点云话题
  -> 建图、定位、避障、导航算法
```

与虚拟机方案的区别：

- 背部主机不需要 VMware 桥接、NAT、虚拟网卡映射。
- 背部主机的有线网口就是接入 M20 的物理接口。
- 背部主机如果还需要远程 SSH/联网，建议 WiFi 用作外网或远程维护，有线口只服务 `10.21.31.0/24` 机器人业务网。

## 2. 硬件与网络前提

### 2.1 外部接口

官方文档中 M20 外部接口提供供电和通信能力。注意：

- M20 对外总输出功率约 `360 W`。
- 其中 `24 V` 供电接口最大输出约 `250 W`。
- `72 V` 对外供电接口实际输出范围约 `56-84 V`。
- 背部主机、散热、交换机、转接模块等总功耗不能超过接口预算。

如果背部主机已经由厂家或硬件同事完成接线，本文只需要关注网络和软件配置。

接线完成后的最小硬件检查：

```bash
# 背部主机上查看有线网口是否有链路
ip -br link
sudo ethtool enp2s0 | egrep 'Speed|Duplex|Link detected'
```

如果系统提示 `sudo: ethtool: command not found`，先不阻塞第一步，可用 `ip -br link` 中的 `LOWER_UP` 作为物理链路已连接的判断依据，后续再安装：

```bash
sudo apt install -y ethtool
```

如果 `Link detected: no`，或 `ip -br link` 没有 `LOWER_UP`，先检查 M20 外部网口、线缆、对外供电开关和背部主机网口状态。调试雷达前应先确认物理链路为 `yes` 或 `LOWER_UP`。

### 2.2 机器狗主机与网段

结合当前 M20 架构与实测，可按下面理解：

| 主机/设备 | 典型地址 | 作用 |
| --- | --- | --- |
| AOS | `10.21.31.103` / `10.21.33.103` / `10.21.32.103` | 上层控制、外设接入、部分官方算法部署 |
| GOS | `10.21.31.104` | 官方建议的用户算法主机之一 |
| NOS | `10.21.31.106` / `10.21.33.106` | 导航/雷达数据链路关键主机，连接雷达网与业务网 |
| 前雷达 | `10.21.33.201` | RoboSense 前雷达 |
| 后雷达 | `10.21.33.202` | RoboSense 后雷达 |
| 背部主机 | 例如 `10.21.31.192` | 我们加装的外部物理 Ubuntu 主机 |

背部主机通常只能直接接入 `10.21.31.0/24`，不能直接接到 `10.21.33.0/24` 雷达网。因此应优先走官方 `multicast-relay.service` 转发方案。

## 3. 背部主机系统环境

### 3.1 ROS2 版本

如果背部主机是 Ubuntu 20.04，建议使用 ROS2 Foxy，和 M20 官方主机环境更接近：

```bash
source /opt/ros/foxy/setup.bash
```

如果背部主机是 Ubuntu 22.04，也可以使用 Humble，但后续要注意 DDS/RMW、消息类型和工作空间依赖兼容。

如果背部主机还没有 ROS2，可先检查：

```bash
ls /opt/ros
```

没有 `foxy` 时，可按官方 ROS2 安装方式安装 Foxy，或参考原 PDF 中的鱼香 ROS 一键安装入口：

```bash
wget http://fishros.com/install -O fishros
bash fishros
```

安装过程中选择 ROS2 Foxy Desktop 版本。安装完成后重新打开终端再执行 `source /opt/ros/foxy/setup.bash`。

### 3.2 必要依赖

```bash
sudo apt update
sudo apt install -y \
  git \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  libyaml-cpp-dev \
  libpcap-dev \
  tcpdump \
  ethtool \
  net-tools
```

如果需要在背部主机直接显示点云：

```bash
sudo apt install -y ros-foxy-rviz2 ros-foxy-pcl-conversions
```

Ubuntu 22.04/Humble 时把 `ros-foxy-*` 替换为 `ros-humble-*`。

### 3.3 ROS2 环境验证

每个新终端先确认 ROS2 环境已经生效：

```bash
source /opt/ros/foxy/setup.bash
printenv ROS_DISTRO
ros2 pkg list | grep -E 'rclcpp|sensor_msgs'
```

建议统一 ROS domain：

```bash
export ROS_DOMAIN_ID=0
echo $ROS_DOMAIN_ID
```

如果背部主机上的雷达驱动、RViz、导航算法都在同一台机器运行，`ROS_DOMAIN_ID=0` 即可。若后续跨主机订阅背部主机发布的话题，所有参与主机的 `ROS_DOMAIN_ID` 必须一致。

## 4. 背部主机有线网口配置

### 4.1 确认网口名

```bash
ip -br addr
ip route
```

例如当前背部主机有线口可能是：

```text
enp2s0  UP  10.21.31.192/24
```

实机记录中，背部主机第一步检查结果为：

```text
enp2s0  UP  10.21.31.192/24
wlx6c1ff78bc632  UP  192.168.112.146/21
default via 192.168.113.1 dev wlx6c1ff78bc632
10.21.31.0/24 dev enp2s0 src 10.21.31.192
ping 10.21.31.106: 4 received, 0% packet loss
```

这表示背部主机有线口已正确进入 M20 `10.21.31.0/24` 业务网，WiFi 仍作为默认路由用于远程维护/外网访问，符合推荐部署方式。

### 4.2 推荐 IP 规划

官方文档示例外部主机 IP 为：

```text
Address: 10.21.31.100
Netmask: 255.255.255.0
Gateway: 10.21.31.1
```

我们当前背部主机可继续使用：

```text
Address: 10.21.31.192
Netmask: 255.255.255.0
```

注意事项：

- IP 必须在 `10.21.31.0/24` 内。
- 不要和 AOS/GOS/NOS/DCU 或其它外设冲突。
- 如果背部主机 WiFi 用于远程 SSH 或联网，有线口建议不要抢默认路由。

### 4.3 NetworkManager 配置方式

如果系统使用 NetworkManager，推荐：

```bash
nmcli dev status
sudo nmcli con add type ethernet ifname enp2s0 con-name m20-wired \
  ipv4.method manual ipv4.addresses 10.21.31.192/24 \
  ipv4.never-default yes ipv6.method ignore
sudo nmcli con up m20-wired
```

如果 `m20-wired` 已存在，不要重复 `add`，直接修改：

```bash
sudo nmcli con mod m20-wired \
  ipv4.method manual ipv4.addresses 10.21.31.192/24 \
  ipv4.never-default yes ipv6.method ignore
sudo nmcli con up m20-wired
```

如果必须配置网关：

```bash
sudo nmcli con mod m20-wired ipv4.gateway 10.21.31.1
sudo nmcli con up m20-wired
```

远程 SSH 通过 WiFi 进入背部主机时，优先保持 `ipv4.never-default yes`，避免有线口改写默认路由导致远程连接中断。

配置后确认：

```bash
ip -br addr show enp2s0
ip route
ping -c 4 10.21.31.106
```

### 4.4 netplan 配置方式

如果系统使用 netplan，可参考：

```yaml
network:
  version: 2
  renderer: NetworkManager
  ethernets:
    enp2s0:
      dhcp4: false
      addresses:
        - 10.21.31.192/24
      optional: true
```

应用：

```bash
sudo netplan apply
```

### 4.5 组播路由与防火墙检查

背部主机同时开 WiFi 和有线口时，Linux 有时会把组播路由选到错误网卡。建议显式确认 `224.10.10.201/202` 走 M20 有线口：

```bash
ip route get 224.10.10.201
ip route get 224.10.10.202
```

实机记录中，未修正前输出为：

```text
multicast 224.10.10.201 dev wlx6c1ff78bc632 src 192.168.112.146
multicast 224.10.10.202 dev wlx6c1ff78bc632 src 192.168.112.146
```

这表示雷达组播被路由到了 WiFi，而不是 M20 有线口 `enp2s0`。此状态下应先修正组播路由，再抓包或启动 SDK。

如果输出没有走 `enp2s0`，临时添加组播路由：

```bash
sudo ip link set dev enp2s0 multicast on
sudo ip route replace 224.0.0.0/4 dev enp2s0
ip route get 224.10.10.201
ip route get 224.10.10.202
```

实机修正后输出为：

```text
Speed: 1000Mb/s
Duplex: Full
Link detected: yes
multicast 224.10.10.201 dev enp2s0 src 10.21.31.192
multicast 224.10.10.202 dev enp2s0 src 10.21.31.192
```

这表示背部主机物理链路和组播路由都已正常。

NetworkManager 持久化可尝试：

```bash
sudo nmcli con mod m20-wired +ipv4.routes "224.0.0.0/4"
sudo nmcli con up m20-wired
```

如果当前 `nmcli` 版本不接受无网关组播路由，就保留临时命令，后续再用 systemd oneshot 固化。

检查防火墙：

```bash
sudo ufw status
```

调试期如果怀疑 UDP 组播被挡，可以临时关闭：

```bash
sudo ufw disable
```

生产环境再按实际安全要求收敛规则。

## 5. 机器人侧开启雷达组播转发

官方文档要求在 NOS `10.21.31.106` 上开启组播转发服务。背部主机先确认能连 NOS：

```bash
ping 10.21.31.106
```

然后 SSH 到 NOS：

```bash
ssh user@10.21.31.106
```

启动并检查服务：

```bash
sudo systemctl enable multicast-relay.service
sudo systemctl start multicast-relay.service
sudo systemctl status multicast-relay.service
sudo journalctl -u multicast-relay.service -f
```

预期状态：

```text
Active: active (running)
Exec: /usr/bin/python3 /usr/bin/multicast.py
```

原 PDF 截图中的服务状态显示 `Tasks: 5`，这与 `/usr/bin/multicast.py` 中四个转发组加主线程基本吻合。当前实机曾看到 `Tasks: 3`，且 `ss` 只显示 `python3` 持有 `7781/7782`，没有持有 `6691/6692`，说明服务虽然 active，但 MSOP 高频转发线程可能没有存活。

如果服务已经 active，单独执行 `sudo systemctl start multicast-relay.service` 通常不会重新初始化线程。遇到只转发 `7781/7782` 的情况，应执行：

```bash
sudo systemctl restart multicast-relay.service
sleep 2
sudo systemctl status multicast-relay.service --no-pager
PID=$(systemctl show -p MainPID --value multicast-relay.service)
ps -Lf -p "$PID"
sudo ss -ulpn | egrep '6691|6692|7781|7782|6681|6682|rslidar|python'
```

说明：

- 该服务负责把雷达相关组播从内部雷达网转发到业务网。
- 官方文档说明该能力依赖 M20 软件版本，通常要求 `V1.1.7` 及以后版本。
- 如非必要，优先只启动/检查官方服务，不修改 AOS/NOS 系统文件。
- `Active: active (running)` 只说明主进程还在，不代表四个转发组都正常。`6691/6692` 是 MSOP 点云主数据，`7781/7782` 是 DIFOP 低频配置/状态数据；只看到 `7781/7782` 时，SDK 仍会因为缺少 MSOP 报超时或无点云。
- 复发的可能原因包括：服务启动早于 `eth0/eth1` 网络完全就绪、组播 membership/IGMP 状态老化、官方 `rslidar` 与 `multicast.py` 的启动顺序导致 MSOP 线程未成功工作，或 `multicast.py` 某个线程异常退出但主进程仍存活。当前先用手动 `restart` 作为安全、低侵入的恢复方法；若后续每天都复发，再考虑 systemd 健康检查或延迟启动。

## 6. 背部主机验证组播数据

在背部主机上抓包：

```bash
sudo timeout 10 tcpdump -ni enp2s0 \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 7781 or port 6692 or port 7782)'
```

如果需要留证据或发给厂家分析，保存 pcap：

```bash
sudo timeout 10 tcpdump -ni enp2s0 -s 0 \
  -w ~/m20_lidar_multicast_test.pcap \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 7781 or port 6692 or port 7782)'
ls -lh ~/m20_lidar_multicast_test.pcap
```

官方方案预期可以看到：

```text
224.10.10.201:6691  前雷达 MSOP 点云数据包
224.10.10.201:7781  前雷达 DIFOP 配置/状态包
224.10.10.202:6692  后雷达 MSOP 点云数据包
224.10.10.202:7782  后雷达 DIFOP 配置/状态包
```

判断标准：

- 能看到 `6691/6692`：点云数据主链路已到背部主机，可以启动 rslidar SDK。
- 只能看到 `7781/7782`：只收到配置/状态包，没有收到点云 MSOP，rslidar SDK 无法生成实时点云。
- 完全看不到：先排查网口 IP、线缆、机器人对外供电、NOS 服务状态。

实机记录中，组播路由修正后背部主机抓包结果为：

```text
10.21.31.106.* > 224.10.10.202.7782: UDP, length 1248
10.21.31.106.* > 224.10.10.201.7781: UDP, length 1248
20 packets captured
0 packets dropped by kernel
```

未看到 `6691/6692`，说明当前只有 DIFOP 被转发到背部主机，MSOP 点云数据尚未到达 `10.21.31.0/24` 业务网。此时不要急着启动 rslidar SDK，应先检查 NOS 侧 `multicast-relay.service` 和 `6691/6692` 端口状态。

建议同时观察包量：

```bash
sudo timeout 5 tcpdump -ni enp2s0 \
  'udp and (dst host 224.10.10.201 or dst host 224.10.10.202)' \
  -c 50
```

正常点云 MSOP 会连续快速出现；如果 5 秒内很少或没有 `6691/6692`，后续 SDK 一般也不会出点云。

## 7. rslidar_sdk 工作空间

### 7.1 新建或使用现有工作空间

如果背部主机已经有 `~/robodog_nav_system` 工作空间，建议直接把包放在该工作空间 `src` 下。

如果希望完全按官方 PDF 复现，也可以新建独立工作空间 `~/rslidar_ws`。本次实机已经采用该方式验证通过：`rslidar_msg` 和 `rslidar_sdk` 均已编译成功，`ros2 pkg executables rslidar_sdk` 能看到 `rslidar_sdk_node`。

从零开始的官方方式：

```bash
mkdir -p ~/rslidar_ws/src
cd ~/rslidar_ws/src
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
cd ..
git clone https://github.com/RoboSense-LiDAR/rslidar_msg.git
```

构建：

```bash
cd ~/rslidar_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

构建后检查包和可执行程序：

```bash
ros2 pkg list | grep -E 'rslidar_sdk|rslidar_msg'
ros2 pkg executables rslidar_sdk
```

如果提示缺依赖，按官方手册补齐：

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions libyaml-cpp-dev libpcap-dev
```

实机构建结果：

```text
rslidar_msg  build finished
rslidar_sdk  build finished
rs_driver Version: v1.5.20
ros2 pkg list | grep rslidar  -> rslidar_msg, rslidar_sdk
ros2 pkg executables rslidar_sdk -> rslidar_sdk rslidar_sdk_node
```

如果使用本项目已经适配过的 `m20_lidar_bridge` 和自定义 `M20_RS48` 解码器，则构建：

```bash
cd ~/robodog_nav_system
source /opt/ros/foxy/setup.bash
colcon build --packages-up-to rslidar_sdk rslidar_msg m20_lidar_bridge --symlink-install
source install/setup.bash
```

### 7.2 雷达型号选择

官方 PDF 示例配置使用：

```yaml
lidar_type: RSAIRY
```

此前我们实测中遇到过 `RSHELIOS` / `RS128` 解码不匹配导致 `ERRCODE_WRONGMSOPBLKID` 的情况。因此背部主机按官方方案复现时建议顺序是：

1. 优先使用官方 PDF 对应的 `RSAIRY` 配置。
2. 如果报 `WRONGMSOPBLKID`，说明 SDK 雷达类型与实际包格式不匹配。
3. 再根据抓包 payload 或厂家确认的具体型号切换解码器，必要时使用我们已适配的自定义解码包。

## 8. rslidar_sdk 配置示例

以下示例按背部主机 IP `10.21.31.192` 改写。若使用其它 IP，请同步替换 `host_address`。官方补充配置里 `host_address: 10.21.33.106` 是 NOS 雷达侧 IP 示例，在背部主机上不能沿用；背部主机应填自己的有线口 IP。

如果直接修改 SDK 原始 `config/config.yaml`，也可以，不必额外新建配置文件。但前后雷达都要显式保留两类地址：

- `host_address`：背部主机本机接收网卡 IP，本机实测为 `10.21.31.192`。
- `group_address`：雷达数据的组播地址，前雷达为 `224.10.10.201`，后雷达为 `224.10.10.202`。

官方资料中“手动添加组播地址”和后续截图/最终 YAML 看起来不一致时，以实际组播链路为准。`rslidar_sdk` 源码中 `group_address` 默认值是 `0.0.0.0`，只有显式配置非 `0.0.0.0` 时才会加入组播组。背部主机接收 NOS 转发出来的 `224.10.10.201/202` 数据时，不建议省略 `group_address`。

建议先复制一份独立配置，避免直接覆盖 SDK 默认文件：

```bash
cd ~/rslidar_ws
cp src/rslidar_sdk/config/config.yaml src/rslidar_sdk/config/m20_backpack_multicast.yaml
vim src/rslidar_sdk/config/m20_backpack_multicast.yaml
```

### 8.1 推荐：前后双雷达同一配置

```yaml
common:
  msg_source: 1
  send_packet_ros: false
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RSAIRY
      msop_port: 6691
      difop_port: 7781
      imu_port: 6681
      host_address: 10.21.31.192
      group_address: 224.10.10.201
      user_layer_bytes: 0
      tail_layer_bytes: 0
      min_distance: 0.2
      max_distance: 200
      use_lidar_clock: true
      dense_points: true
      ts_first_point: true
      start_angle: 0
      end_angle: 360
      pcap_repeat: true
      pcap_rate: 1
      pcap_path: /home/robosense/lidar.pcap
      x: 0.0501
      'y': 0
      z: 0.739
      roll: 0
      pitch: 1.570795
      yaw: 0
    ros:
      ros_frame_id: rslidar_front
      ros_recv_packet_topic: /rslidar_packets_front
      ros_send_packet_topic: /rslidar_packets_front
      ros_send_imu_data_topic: /rslidar_imu_data_front
      ros_send_point_cloud_topic: /rslidar_points_front
      ros_queue_length: 100

  - driver:
      lidar_type: RSAIRY
      msop_port: 6692
      difop_port: 7782
      imu_port: 6682
      host_address: 10.21.31.192
      group_address: 224.10.10.202
      user_layer_bytes: 0
      tail_layer_bytes: 0
      min_distance: 0.2
      max_distance: 60
      use_lidar_clock: true
      dense_points: true
      ts_first_point: true
      start_angle: 0
      end_angle: 360
      pcap_repeat: true
      pcap_rate: 1
      pcap_path: /home/robosense/lidar.pcap
      x: -0.32028
      'y': 0
      z: -0.013
      roll: 0
      pitch: -1.57079
      yaw: 0
    ros:
      ros_frame_id: rslidar_rear
      ros_recv_packet_topic: /rslidar_packets_rear
      ros_send_packet_topic: /rslidar_packets_rear
      ros_send_imu_data_topic: /rslidar_imu_data_rear
      ros_send_point_cloud_topic: /rslidar_points_rear
      ros_queue_length: 100
```

如果 `RSAIRY` 报 `ERRCODE_WRONGMSOPBLKID`，先不要改其它网络参数，优先切换 `lidar_type` 或使用已适配的自定义 decoder 验证。

### 8.2 前雷达单独配置

```yaml
common:
  msg_source: 1
  send_packet_ros: false
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RSAIRY
      msop_port: 6691
      difop_port: 7781
      imu_port: 6681
      host_address: 10.21.31.192
      group_address: 224.10.10.201
      user_layer_bytes: 0
      tail_layer_bytes: 0
      min_distance: 0.2
      max_distance: 200
      use_lidar_clock: true
      dense_points: true
      ts_first_point: true
      start_angle: 0
      end_angle: 360
      pcap_repeat: true
      pcap_rate: 1
      pcap_path: /home/robosense/lidar.pcap
      x: 0.0501
      'y': 0
      z: 0.739
      roll: 0
      pitch: 1.570795
      yaw: 0
    ros:
      ros_frame_id: rslidar_front
      ros_recv_packet_topic: /rslidar_packets_front
      ros_send_packet_topic: /rslidar_packets_front
      ros_send_imu_data_topic: /rslidar_imu_data_front
      ros_send_point_cloud_topic: /rslidar_points_front
      ros_queue_length: 100
```

### 8.3 后雷达单独配置

```yaml
common:
  msg_source: 1
  send_packet_ros: false
  send_point_cloud_ros: true

lidar:
  - driver:
      lidar_type: RSAIRY
      msop_port: 6692
      difop_port: 7782
      imu_port: 6682
      host_address: 10.21.31.192
      group_address: 224.10.10.202
      user_layer_bytes: 0
      tail_layer_bytes: 0
      min_distance: 0.2
      max_distance: 60
      use_lidar_clock: true
      dense_points: true
      ts_first_point: true
      start_angle: 0
      end_angle: 360
      pcap_repeat: true
      pcap_rate: 1
      pcap_path: /home/robosense/lidar.pcap
      x: -0.32028
      'y': 0
      z: -0.013
      roll: 0
      pitch: -1.57079
      yaw: 0
    ros:
      ros_frame_id: rslidar_rear
      ros_recv_packet_topic: /rslidar_packets_rear
      ros_send_packet_topic: /rslidar_packets_rear
      ros_send_imu_data_topic: /rslidar_imu_data_rear
      ros_send_point_cloud_topic: /rslidar_points_rear
      ros_queue_length: 100
```

实际部署时优先使用 8.1 的双雷达配置，让一个 rslidar_sdk 节点同时接前后雷达。前后雷达外参 `x/y/z/roll/pitch/yaw` 应按 M20 实际安装位姿修正，不建议长期使用全零外参。

## 9. 启动雷达驱动

官方启动方式：

```bash
cd ~/rslidar_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rslidar_sdk start.py
```

注意：官方 `start.py` 会同时启动 SDK 和 RViz，但它自带的 `rviz/rviz2.rviz` 是单雷达默认配置：

```text
Fixed Frame: rslidar
PointCloud2 Topic: /rslidar_points
```

而本次 M20 双雷达配置发布的是：

```text
/rslidar_points_front  frame_id: rslidar_front
/rslidar_points_rear   frame_id: rslidar_rear
```

因此直接使用 `ros2 launch rslidar_sdk start.py` 打开的 RViz 可能提示 `Frame [rslidar] does not exist`，且默认 PointCloud2 话题也不是我们实际发布的话题。这不是点云链路坏了，而是 RViz 默认配置与当前双雷达配置不匹配。

原手册 5.1 是“启动雷达组播转发服务”，对应机器人侧 `multicast-relay.service`；5.2 才是启动 `rslidar_sdk`。当前实机已经确认服务是 active/running，但只有 `7781/7782` 转发出来，`6691/6692` 没有转发到背部主机，因此应回到 5.1 对该服务做状态核查和必要的 restart。

如果直接使用原手册 5.2 的 `ros2 launch rslidar_sdk start.py`，一定要看启动日志里的 `Config loaded from PATH`，确认它读取的是刚修改过的配置，并打印出：

```text
host_address: 10.21.31.192
group_address: 224.10.10.201
group_address: 224.10.10.202
```

为避免 launch 文件读到安装目录里的旧配置，调试期更推荐显式指定 `config_path`。

如果使用自定义配置文件，推荐直接指定配置路径：

```bash
ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:=/home/m20/rslidar_ws/src/rslidar_sdk/config/m20_backpack_multicast.yaml
```

本次实机直接修改了 SDK 原始 `config/config.yaml`，启动命令为：

```bash
cd ~/rslidar_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:=/home/m20/rslidar_ws/src/rslidar_sdk/config/config.yaml
```

实机启动日志已确认配置被正确读取：

```text
Config loaded from PATH:
/home/m20/rslidar_ws/src/rslidar_sdk/config/config.yaml
lidar_type: RSAIRY
host_address: 10.21.31.192
group_address: 224.10.10.201
PointCloud Topic: /rslidar_points_front
group_address: 224.10.10.202
PointCloud Topic: /rslidar_points_rear
RoboSense-LiDAR-Driver is running.....
```

日志中 `imu_port` 打印为 `0` 是因为当前 SDK 编译未启用 `ENABLE_IMU_DATA_PARSE`，不影响先验证 `PointCloud2` 点云链路。

如果使用本项目已适配的 `M20_RS48` 解码器：

```bash
ros2 run m20_lidar_bridge run_backpack_m20_rs48
```

启动成功时应看到类似：

```text
RSLidar SDK Version: v1.5.x
Receive Packets From : Online LiDAR
Msop Port: 6691
Difop Port: 7781
Send PointCloud To : ROS
PointCloud Topic: /rslidar_points_front
RoboSense-LiDAR-Driver is running.....
```

## 10. 点云话题验证

另开终端：

```bash
cd ~/rslidar_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 topic list | grep rslidar
```

检查频率：

```bash
ros2 topic hz /rslidar_points_front
ros2 topic hz /rslidar_points_rear
```

检查消息类型和 QoS：

```bash
ros2 topic info -v /rslidar_points_front
ros2 topic info -v /rslidar_points_rear
```

检查点云头部和字段：

```bash
python3 - /rslidar_points_front <<'PY'
import sys
import rclpy
from sensor_msgs.msg import PointCloud2

topic = sys.argv[1]

def cb(msg):
    print(f'topic: {topic}')
    print(f'frame_id: {msg.header.frame_id}')
    print(f'stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
    print(f'height: {msg.height}, width: {msg.width}, point_step: {msg.point_step}, row_step: {msg.row_step}, is_dense: {msg.is_dense}')
    print('fields:')
    for f in msg.fields:
        print(f'  {f.name}: offset={f.offset}, datatype={f.datatype}, count={f.count}')
    rclpy.shutdown()

rclpy.init()
node = rclpy.create_node('pointcloud2_probe')
sub = node.create_subscription(PointCloud2, topic, cb, 10)
rclpy.spin(node)
PY

python3 - /rslidar_points_rear <<'PY'
import sys
import rclpy
from sensor_msgs.msg import PointCloud2

topic = sys.argv[1]

def cb(msg):
    print(f'topic: {topic}')
    print(f'frame_id: {msg.header.frame_id}')
    print(f'stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')
    print(f'height: {msg.height}, width: {msg.width}, point_step: {msg.point_step}, row_step: {msg.row_step}, is_dense: {msg.is_dense}')
    print('fields:')
    for f in msg.fields:
        print(f'  {f.name}: offset={f.offset}, datatype={f.datatype}, count={f.count}')
    rclpy.shutdown()

rclpy.init()
node = rclpy.create_node('pointcloud2_probe')
sub = node.create_subscription(PointCloud2, topic, cb, 10)
rclpy.spin(node)
PY
```

ROS2 Foxy 的 `ros2 topic echo` 不一定支持 `--once` 和 `--field`，因此这里使用 Python one-shot subscriber。不要长时间直接 `echo` 完整点云，因为 `PointCloud2` 的 `data` 数组很大。

如果 `ros2 topic hz` 长时间没有输出，保持 SDK 运行，在另一个终端同时确认组播 membership 和网卡实际收包：

```bash
ip maddr show dev enp2s0 | grep -E '224.10.10.201|224.10.10.202' || true

sudo timeout 8 tcpdump -ni enp2s0 \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 6692 or port 7781 or port 7782)'
```

若此时仍只有 `7781/7782`，说明即使 SDK 已加入组播组，背部主机仍没有收到 MSOP 点云数据，问题应继续回到 NOS `multicast-relay.service` 的 MSOP 转发。

实机进一步验证结果：

```text
ros2 topic list | grep rslidar
/rslidar_points_front
/rslidar_points_rear

ros2 topic hz /rslidar_points_front  # 长时间无频率输出
ros2 topic hz /rslidar_points_rear   # 长时间无频率输出

ip maddr show dev enp2s0
inet 224.10.10.202 users 2
inet 224.10.10.201 users 2

tcpdump enp2s0:
10.21.31.106.* > 224.10.10.202.7782
10.21.31.106.* > 224.10.10.201.7781
```

这说明：

- SDK 已经创建 ROS2 点云发布者，所以 `topic list` 能看到 `/rslidar_points_front` 和 `/rslidar_points_rear`。
- 但没有 MSOP 点云包进入 SDK，所以 `topic hz` 没有实际消息频率。
- 组播 membership 已建立，背部主机侧配置和加入组播动作已基本排除。
- 当前瓶颈继续锁定在 NOS `eth0 -> eth1` 的 `6691/6692` MSOP 转发。

录制一小段测试包：

```bash
mkdir -p ~/bags
ros2 bag record -o ~/bags/m20_lidar_smoke_test \
  /rslidar_points_front /rslidar_points_rear
```

回放验证：

```bash
ros2 bag info ~/bags/m20_lidar_smoke_test
ros2 bag play ~/bags/m20_lidar_smoke_test
```

确认测试包可回放后，用 `Ctrl+C` 结束录制和回放进程。

RViz 显示：

```bash
rviz2
```

添加 `PointCloud2`，话题选择 `/rslidar_points_front` 或 `/rslidar_points_rear`。先用下面方式验证单路点云：

```text
前雷达：
Fixed Frame = rslidar_front
PointCloud2 Topic = /rslidar_points_front

后雷达：
Fixed Frame = rslidar_rear
PointCloud2 Topic = /rslidar_points_rear
```

如果 RViz 报 `Frame [rslidar] does not exist`，通常是仍在使用官方默认 RViz 配置。把 Fixed Frame 从 `rslidar` 改成当前点云消息头里的 `frame_id` 即可。可用下面命令确认：

```bash
python3 - /rslidar_points_front <<'PY'
import sys
import rclpy
from sensor_msgs.msg import PointCloud2
topic = sys.argv[1]
def cb(msg):
    print(msg.header)
    rclpy.shutdown()
rclpy.init()
node = rclpy.create_node('pointcloud2_header_probe')
sub = node.create_subscription(PointCloud2, topic, cb, 10)
rclpy.spin(node)
PY
```

后续接入机器人 TF 后再统一到 `base_link` / `map`。

如果 RViz 需要临时固定到 `base_link`，可以先发静态 TF 占位：

```bash
# 终端 1
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link rslidar_front
```

```bash
# 终端 2
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link rslidar_rear
```

这只是显示用占位，真实导航必须替换为准确的雷达到机身外参。

## 11. 接入导航算法的建议

背部主机拿到前后雷达点云后，建议分三层接入导航：

### 11.1 原始点云层

保留：

```text
/rslidar_points_front
/rslidar_points_rear
```

用于录包、建图、三维定位、调试和离线分析。

### 11.2 融合/裁剪层

新增我们自己的中间话题：

```text
/m20/lidar/front/points_roi
/m20/lidar/rear/points_roi
/m20/lidar/points_fused
```

建议做：

- ROI 裁剪，去掉车体、自身腿部、天空和过远点。
- voxel 降采样，降低 CPU 和网络压力。
- 前后雷达外参变换到 `base_link`。
- 可选地根据地面高度切出障碍云。

### 11.3 导航消费层

短期仍可服务 Nav2/MPPI：

```text
三维点云 -> 地面/障碍提取 -> 2D LaserScan 或 2D costmap obstacle layer
```

中长期可服务三维规划：

```text
三维点云/PCD 地图 -> 三维定位/建图 -> 3D/2.5D 避障规划 -> M20 底盘速度接口
```

M20 是轮足式，路径后端还应区分：

- 曲率小、空间充足：优先轮式平顺运动。
- 急转弯、狭窄空间、原地调整：再调用更偏腿式/复合运动的能力。

## 12. 可选：背部主机 systemd 自启动

稳定后可把驱动做成服务。示例：

```ini
[Unit]
Description=M20 Backpack RoboSense LiDAR Driver
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=m20
WorkingDirectory=/home/m20/rslidar_ws
Environment=ROS_DOMAIN_ID=0
ExecStart=/bin/bash -lc 'source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:=/home/m20/rslidar_ws/src/rslidar_sdk/config/config.yaml'
Restart=on-failure
RestartSec=3

[Install]
WantedBy=multi-user.target
```

保存为：

```bash
sudo vim /etc/systemd/system/m20-rslidar.service
sudo systemctl daemon-reload
sudo systemctl enable m20-rslidar.service
sudo systemctl start m20-rslidar.service
sudo systemctl status m20-rslidar.service
```

查看运行日志：

```bash
sudo journalctl -u m20-rslidar.service -f
```

调试期不建议一上来就自启动，先手动确认抓包、解码、ROS 话题、RViz 都正常。

注意：组播路由请按第 4.5 节在网络配置阶段处理，不建议依赖雷达驱动服务临时修改系统路由。

## 13. 常见问题排查

### 13.1 ping 不通 `10.21.31.106`

检查：

```bash
ip -br addr
ip route
nmcli dev status
```

重点：

- 背部主机有线口是否为 `10.21.31.x/24`。
- 网线是否接到 M20 外部网口。
- 机器狗对外供电/通信口是否启用。
- IP 是否冲突。

### 13.2 能 ping，但抓不到 `6691/6692`

先看机器人侧服务：

```bash
ssh user@10.21.31.106
sudo systemctl status multicast-relay.service
sudo journalctl -u multicast-relay.service -f
```

再在 NOS 上检查端口占用：

```bash
sudo ss -ulpn | egrep '6691|6692|7781|7782|multicast|rslidar'
```

如果背部主机只能看到 `7781/7782`，看不到 `6691/6692`，说明只转发到了 DIFOP，没有转发 MSOP。此时 rslidar_sdk 启动后通常会超时或没有点云。可尝试：

- 重启 `multicast-relay.service`。
- 确认 M20 软件版本是否满足官方文档要求。
- 重启机器人后再次验证。
- 如果官方服务仍不转发 MSOP，再考虑临时 raw relay 或让厂家确认服务配置。

实机进一步在 NOS 上验证：

```text
NOS eth0 = 10.21.33.106/24
NOS eth1 = 10.21.31.106/24
multicast-relay.service active (running), /usr/bin/python3 /usr/bin/multicast.py
rslidar(pid=3191) 占用 224.10.10.201:6691 和 224.10.10.202:6692
python3(multicast.py) 占用 224.10.10.201:7781 和 224.10.10.202:7782
NOS eth0 抓包可见 10.21.33.201/202 -> 224.10.10.201/202:6691/6692
NOS eth1 抓包只可见 10.21.31.106 -> 224.10.10.201/202:7781/7782
```

该状态说明：MSOP 点云包已到达 NOS 的雷达网 `eth0`，但未从 NOS 转发到业务网 `eth1`。背部主机当前无法靠官方转发链路直接收到 `6691/6692`，需要继续检查 `multicast.py` 和官方 `rslidar` 进程的端口占用关系，或改用不抢占端口的原始包转发方案。

进一步读取 `/usr/bin/multicast.py` 后确认，官方脚本本身配置了四组转发：

```python
GROUPS = [
    ("224.10.10.202", 6692),
    ("224.10.10.202", 7782),
    ("224.10.10.201", 6691),
    ("224.10.10.201", 7781),
]
RECV_IFACE = "10.21.33.106"
SEND_IFACE = "10.21.31.106"
```

但实机 `systemctl status` 显示该服务只有 `Tasks: 3`，`ss -ulpn` 中 `python3` 只实际持有 `7781/7782` 及对应发送端口，没有持有 `6691/6692`。这说明当前不是配置文件没写 MSOP，而是 `6691/6692` 对应转发线程没有存活或没有成功绑定。由于官方 `rslidar` 进程同时持有 `224.10.10.201:6691` 和 `224.10.10.202:6692`，下一步应验证是否存在端口绑定冲突，再决定是重启官方转发服务、联系厂家确认，还是使用不绑定 `6691/6692` 的 raw relay 方案。

绑定探针实机结果：

```text
OK 224.10.10.202 6692
OK 224.10.10.202 7782
OK 224.10.10.201 6691
OK 224.10.10.201 7781
```

这说明当前系统允许其它进程使用 `SO_REUSEADDR` 绑定这些组播端口，问题不再是简单的端口完全不可绑定。结合 `Tasks: 3` 和实际抓包现象，更像是 `multicast.py` 的 MSOP 高频转发线程未存活、未运行或运行后异常退出。背部主机可先按官方 PDF 第四步准备 rslidar_sdk 工作空间，但只有在 `6691/6692` 到达背部主机后，SDK 才能真正发布实时点云。

### 13.3 `ERRCODE_WRONGMSOPBLKID`

含义通常是雷达类型配置不匹配。处理顺序：

1. 使用官方文档示例的 `RSAIRY`。
2. 核对 SDK 版本和车辆实际雷达型号。
3. 抓取少量 pcap，检查 payload 格式。
4. 必要时切换到已适配的自定义 decoder。

如果本项目 `m20_lidar_bridge` 已在背部主机构建过，可直接试：

```bash
source /opt/ros/foxy/setup.bash
source ~/robodog_nav_system/install/setup.bash
ros2 run m20_lidar_bridge run_backpack_m20_rs48
```

### 13.4 `ERRCODE_MSOPTIMEOUT`

含义是 rslidar SDK 在超时时间内没有收到 MSOP 点云数据包。结合本次实机现象：

```text
/rslidar_points_front
/rslidar_points_rear
tcpdump enp2s0 只有 7781/7782，没有 6691/6692
```

该错误不是 ROS2 话题发现问题，也不是 `group_address` 没加入；更像是 NOS 的 `multicast-relay.service` 没有把 `6691/6692` 从 `eth0` 雷达网转发到 `eth1` 业务网。

先在 NOS 上按原手册 5.1 思路重启转发服务：

```bash
sudo systemctl restart multicast-relay.service
sleep 2
sudo systemctl status multicast-relay.service --no-pager
PID=$(systemctl show -p MainPID --value multicast-relay.service)
ps -Lf -p "$PID"
sudo ss -ulpn | egrep '6691|6692|7781|7782|6681|6682|rslidar|python'
```

然后马上确认 NOS `eth1` 是否开始转发 MSOP：

```bash
sudo timeout 5 tcpdump -ni eth1 \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 6692 or port 7781 or port 7782)'
```

成功标志是能看到 `224.10.10.201:6691` 和 `224.10.10.202:6692` 高频出现。若重启后仍只有 `7781/7782`，应抓取服务日志中的 Python 异常，或临时前台运行 `/usr/bin/multicast.py` 观察 MSOP 线程为什么退出。

实机重启 `multicast-relay.service` 后恢复正常：

```text
Active: active (running)
Tasks: 5
python3(pid=509388) 持有 224.10.10.201:6691
python3(pid=509388) 持有 224.10.10.202:6692
python3(pid=509388) 持有 224.10.10.201:7781
python3(pid=509388) 持有 224.10.10.202:7782

tcpdump eth1:
10.21.31.106.* > 224.10.10.202.6692
10.21.31.106.* > 224.10.10.201.6691
22096 packets captured / 5s
0 packets dropped
```

这说明 NOS 侧 MSOP 转发已经恢复。下一步应回到背部主机确认 `enp2s0` 能收到 `6691/6692`，然后重启 rslidar SDK 验证点云频率。

背部主机进一步抓包确认：

```text
tcpdump enp2s0:
10.21.31.106.* > 224.10.10.201.6691
10.21.31.106.* > 224.10.10.202.6692
22145 packets captured / 5s
28 packets dropped
```

这说明 MSOP 点云主数据已经到达背部主机。若混合抓 `6691/6692/7781/7782` 时没有明显看到 `7781/7782`，不要直接判定 DIFOP 丢失，因为 MSOP 频率很高，会快速刷屏。应分别抓：

```bash
sudo timeout 6 tcpdump -ni enp2s0 \
  'udp and (dst port 7781 or dst port 7782)' \
  -c 10

sudo timeout 3 tcpdump -ni enp2s0 \
  'udp and (dst port 6691 or dst port 6692)' \
  -c 20
```

其中：

- `6691/6692` 是 MSOP，包含实际测距点云数据，是生成 `PointCloud2` 的主数据流。
- `7781/7782` 是 DIFOP，包含雷达设备信息、状态、配置、标定/角度、回波模式等低频信息。rslidar SDK 当前打印 `wait_for_difop: 1`，因此如果长期收不到 DIFOP，可能仍然不发布稳定点云。

实机已确认 DIFOP 低频包也到达背部主机：

```text
10.21.31.106.* > 224.10.10.202.7782
10.21.31.106.* > 224.10.10.201.7781
```

随后重启 rslidar SDK，前雷达点云话题已稳定发布：

```text
ros2 topic hz /rslidar_points_front
average rate: 10.003
average rate: 10.002
average rate: 10.001
```

到此为止，背部主机接收前雷达 `PointCloud2` 的链路已经跑通。还需继续确认 `/rslidar_points_rear`、点云字段、RViz 显示和服务重启后的持久性。

### 13.5 有 ROS 话题但频率低或丢包

检查：

```bash
ros2 topic hz /rslidar_points_front
top
sudo ethtool -S enp2s0
```

可调大 socket buffer：

```bash
sudo sysctl -w net.core.rmem_max=268435456
sudo sysctl -w net.core.rmem_default=268435456
```

长期配置可写入 `/etc/sysctl.d/99-m20-lidar.conf`。

### 13.6 RViz 没有点云

检查：

- Fixed Frame 是否设置为点云的 `frame_id`。
- Foxy 兼容的 Python one-shot subscriber 是否能读到点云 `header.frame_id`。
- RViz 是否在同一个 ROS_DOMAIN_ID 和同一个 ROS2 环境中。
- 点云是否被外参变换到了视野外。

## 14. 推荐落地顺序

1. 背部主机有线口固定到 `10.21.31.x/24`，本次为 `10.21.31.192`。
2. 背部主机确认组播路由走有线口：

```bash
sudo ip route replace 224.0.0.0/4 dev enp2s0
ip route get 224.10.10.201
ip route get 224.10.10.202
```

3. 背部主机 ping 通 NOS：

```bash
ping 10.21.31.106
```

4. NOS 检查并必要时重启组播转发服务：

```bash
ssh user@10.21.31.106
sudo systemctl restart multicast-relay.service
sleep 2
sudo systemctl status multicast-relay.service --no-pager
PID=$(systemctl show -p MainPID --value multicast-relay.service)
ps -Lf -p "$PID"
sudo ss -ulpn | egrep '6691|6692|7781|7782|python'
```

5. NOS `eth1` 确认 `6691/6692` 已转发到业务网：

```bash
sudo timeout 5 tcpdump -ni eth1 \
  'udp and (host 224.10.10.201 or host 224.10.10.202) and (port 6691 or port 6692 or port 7781 or port 7782)'
```

6. 背部主机确认四个端口都能看到：

```bash
sudo timeout 3 tcpdump -ni enp2s0 \
  'udp and (dst port 6691 or dst port 6692)' \
  -c 20

sudo timeout 6 tcpdump -ni enp2s0 \
  'udp and (dst port 7781 or dst port 7782)' \
  -c 10
```

7. 背部主机启动 rslidar SDK：

```bash
cd ~/rslidar_ws
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
  -p config_path:=/home/m20/rslidar_ws/src/rslidar_sdk/config/config.yaml
```

8. 确认前后点云稳定频率：

```bash
ros2 topic hz /rslidar_points_front
ros2 topic hz /rslidar_points_rear
```

9. 确认消息头和字段：

```bash
python3 - /rslidar_points_front <<'PY'
import sys
import rclpy
from sensor_msgs.msg import PointCloud2
topic = sys.argv[1]
def cb(msg):
    print(f'frame_id: {msg.header.frame_id}')
    print(f'height: {msg.height}, width: {msg.width}')
    print([(f.name, f.offset, f.datatype, f.count) for f in msg.fields])
    rclpy.shutdown()
rclpy.init()
node = rclpy.create_node('pointcloud2_probe')
sub = node.create_subscription(PointCloud2, topic, cb, 10)
rclpy.spin(node)
PY
```

10. RViz 显示点云，录制 rosbag 作为算法开发基准数据。
11. 接入 ROI/fusion/TF/导航中间层。
12. 稳定后再做背部主机 `rslidar_sdk` 的 systemd 自启动；NOS 侧转发服务保持官方服务，但要纳入健康检查。
