# ============================================================================
# 文件：rslidar_pointcloud_to_scan.launch.py
# 功能：启动 pointcloud_to_laserscan 节点，把 RS-LiDAR 的 3D 点云
#       （默认话题 /LIDAR/POINTS）转换为二维激光扫描（默认话题 /scan）。
#       通过高度裁剪（min_height/max_height）、角度范围（angle_min/max）、
#       距离范围（range_min/max）等参数，得到 Nav2 可直接使用的 2D 数据。
#       本文件常被其他 launch 包含复用（如 3D 雷达仿真模式、PCD 回放验证等）。
# ============================================================================

from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument  # 声明可配置的 launch 参数
from launch.substitutions import LaunchConfiguration  # 运行期替换：引用 launch 参数（命令行传入或默认值）的值
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点
from launch_ros.parameter_descriptions import ParameterValue  # 把 launch 参数显式转换为指定 ROS2 参数类型


def generate_launch_description():
    # —— 声明 launch 参数（供本文件内引用，可被包含方/命令行覆盖）——
    use_sim_time = LaunchConfiguration("use_sim_time")  # 是否使用仿真时间
    cloud_topic = LaunchConfiguration("cloud_topic")  # 输入点云话题
    scan_topic = LaunchConfiguration("scan_topic")  # 输出 2D 扫描话题
    target_frame = LaunchConfiguration("target_frame")  # 目标坐标系（空字符串则使用点云自身坐标系）
    min_height = LaunchConfiguration("min_height")  # 裁剪高度下限（米，过滤地面以下/贴近地面的点）
    max_height = LaunchConfiguration("max_height")  # 裁剪高度上限（米，过滤高处点如车身/天花板）
    angle_min = LaunchConfiguration("angle_min")  # 扫描起始角度（弧度）
    angle_max = LaunchConfiguration("angle_max")  # 扫描结束角度（弧度）
    angle_increment = LaunchConfiguration("angle_increment")  # 角度分辨率（弧度/束）
    scan_time = LaunchConfiguration("scan_time")  # 每帧扫描间隔（秒，决定扫描频率）
    range_min = LaunchConfiguration("range_min")  # 最小量程（米，跳过机器人自身/近处杂点）
    range_max = LaunchConfiguration("range_max")  # 最大量程（米）
    use_inf = LaunchConfiguration("use_inf")  # 无回波的角度是否用 inf（无穷大）填充，保持角度连续
    transform_tolerance = LaunchConfiguration("transform_tolerance")  # TF 变换容差（秒）

    return LaunchDescription([
        # —— 声明参数默认值 ——
        DeclareLaunchArgument("use_sim_time", default_value="true"),  # 默认使用仿真时间
        DeclareLaunchArgument("cloud_topic", default_value="/LIDAR/POINTS"),  # 默认输入话题：RS-LiDAR 点云
        DeclareLaunchArgument("scan_topic", default_value="/scan"),  # 默认输出话题：Nav2 标准扫描话题
        DeclareLaunchArgument("target_frame", default_value=""),  # 默认不指定目标坐标系
        DeclareLaunchArgument("min_height", default_value="-0.30"),  # 过滤高度 < -0.30m 的点（地面以下杂点）
        DeclareLaunchArgument("max_height", default_value="0.30"),  # 过滤高度 > 0.30m 的点（高处干扰）
        DeclareLaunchArgument("angle_min", default_value="-3.141592653589793"),  # 起始角度 -π（全向 360° 扫描）
        DeclareLaunchArgument("angle_max", default_value="3.141592653589793"),  # 结束角度 +π（全向 360° 扫描）
        DeclareLaunchArgument("angle_increment", default_value="0.0034906585"),  # 角度分辨率约 0.2°（约 1080 束/圈）
        DeclareLaunchArgument("scan_time", default_value="0.10"),  # 扫描间隔 0.1 秒（约 10Hz）
        DeclareLaunchArgument("range_min", default_value="0.20"),  # 最小量程 0.2m（跳过机器人自身点）
        DeclareLaunchArgument("range_max", default_value="30.0"),  # 最大量程 30m
        DeclareLaunchArgument("use_inf", default_value="true"),  # 无回波角度用 inf 填充（保证扫描角度连续）
        DeclareLaunchArgument("transform_tolerance", default_value="0.01"),  # TF 容差 10ms

        # —— 启动点云→激光扫描转换节点 ——
        Node(  # 启动 pointcloud_to_laserscan 节点，将 3D 点云投影为 2D 扫描
            package="pointcloud_to_laserscan",  # 所属功能包
            executable="pointcloud_to_laserscan_node",  # 可执行文件名
            name="rslidar_pointcloud_to_scan",  # 节点名（与传感器对应）
            output="screen",  # 日志输出到屏幕
            remappings=[  # 话题重映射
                ("cloud_in", cloud_topic),  # 输入点云话题 → 节点内部话题 cloud_in
                ("scan", scan_topic),  # 节点内部话题 scan → 输出扫描话题
            ],
            parameters=[{  # 节点参数（用 ParameterValue 显式指定类型，保证 ROS2 参数类型正确）
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),  # 仿真时间（布尔型）
                "target_frame": ParameterValue(target_frame, value_type=str),  # 目标坐标系（字符串型）
                "transform_tolerance": ParameterValue(transform_tolerance, value_type=float),  # TF 容差（浮点型）
                "min_height": ParameterValue(min_height, value_type=float),  # 高度下限（浮点型）
                "max_height": ParameterValue(max_height, value_type=float),  # 高度上限（浮点型）
                "angle_min": ParameterValue(angle_min, value_type=float),  # 起始角度（浮点型）
                "angle_max": ParameterValue(angle_max, value_type=float),  # 结束角度（浮点型）
                "angle_increment": ParameterValue(angle_increment, value_type=float),  # 角度分辨率（浮点型）
                "scan_time": ParameterValue(scan_time, value_type=float),  # 扫描间隔（浮点型）
                "range_min": ParameterValue(range_min, value_type=float),  # 最小量程（浮点型）
                "range_max": ParameterValue(range_max, value_type=float),  # 最大量程（浮点型）
                "use_inf": ParameterValue(use_inf, value_type=bool),  # 是否用 inf 填充（布尔型）
            }],
        ),
    ])
