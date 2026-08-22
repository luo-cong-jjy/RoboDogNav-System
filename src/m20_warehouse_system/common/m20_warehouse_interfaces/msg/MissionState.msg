# Typed state of the active warehouse inspection mission.
# 消息用途：报告"当前激活的仓库巡检任务"的详细状态（供监控界面/调度使用）
std_msgs/Header header       # 标准消息头
string mission_id            # 任务 ID
string state                 # 任务总状态（IDLE / RUNNING / PAUSED / FAULT / COMPLETED 等）
uint32 step_index            # 当前执行到的步骤序号（从 0 开始）
uint32 step_count            # 任务总步骤数
string step_type             # 当前步骤类型（如 NAVIGATE / INSPECT / TRANSFER）
string step_name             # 当前步骤名称（人类可读）
string active_floor          # 当前任务所在的楼层
uint64 map_generation        # 当前使用的楼层地图代数
bool paused                  # 任务是否被暂停
bool fault                   # 任务是否处于故障状态
string message               # 附加说明/错误信息
