# Atomic state of the active floor map.
# 消息用途：原子地报告"当前激活楼层地图"的状态（楼层切换/地图重建时使用）
std_msgs/Header header       # 标准消息头
string floor_id              # 当前激活的楼层 ID
uint64 generation            # 地图代数：每次地图重建递增，用于判断状态是否过期
bool ready                   # 地图是否已就绪（可用于导航）
string phase                 # 当前生命周期阶段（如 LOADING / GENERATING / READY / FAILED）
string message               # 附加说明/错误信息
