# Typed state of the single active navigation goal.
# 消息用途：报告"当前唯一激活的导航目标点"的状态（保证任一时刻只有一个活动目标）
std_msgs/Header header       # 标准消息头
string goal_id               # 导航目标 ID
string floor_id              # 目标所在楼层 ID
uint64 map_generation        # 目标所属的地图代数（地图切换后旧目标应失效）
string state                 # 导航状态（ACTIVE / ARRIVED / FAILED / CANCELLED 等）
bool active                  # 目标是否仍在执行中
float32 distance_remaining   # 距离目标的剩余距离 [m]
string message               # 附加说明/错误信息
