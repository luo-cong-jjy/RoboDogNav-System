# Readiness of local sensing data derived from the active map generation.
# 消息用途：报告"基于当前激活楼层地图生成的局部感知数据"的就绪状态
std_msgs/Header header       # 标准消息头：时间戳 + frame_id
string floor_id              # 当前楼层 ID（数据来源楼层）
uint64 generation            # 地图代数（generation）：同一楼层地图每次重建递增，用于识别数据新旧
bool ready                   # 局部感知数据是否就绪（true = 可被下游导航/安全模块消费）
uint32 fresh_cloud_count     # 就绪以来收到的新鲜点云帧数（用于监控数据流是否中断）
uint32 point_count           # 当前局部点云中的点数（体素下采样后）
float32 min_x                # 局部点云包围盒 X 最小值 [m]
float32 max_x                # 局部点云包围盒 X 最大值 [m]
float32 min_y                # 局部点云包围盒 Y 最小值 [m]
float32 max_y                # 局部点云包围盒 Y 最大值 [m]
string message               # 附加说明/错误信息（就绪为 false 时描述原因）
