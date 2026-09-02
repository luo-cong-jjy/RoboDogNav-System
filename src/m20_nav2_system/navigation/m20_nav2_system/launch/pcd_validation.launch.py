# ============================================================================
# 文件：pcd_validation.launch.py
# 功能：PCD（点云文件）验证 launch：回放一个离线 PCD 文件，验证
#       「点云 → 2D 扫描 → Nav2 costmap」整条链路是否工作正常。
#       流程：
#         - 发布静态 TF（odom→base_link→lidar_link，回放模式用单位变换占位）
#         - 包含点云转扫描 launch，把切片点云转为 /scan
#         - 启动 Nav2 costmap（nav2_costmap_2d，加载 nav2_costmap_from_bag.yaml）
#         - 2 秒后通过 lifecycle_configure_activate 激活 costmap 生命周期节点
#         - 7 秒后启动 pcd_slice_publisher 把 PCD 切片为 2D 点云逐帧发布
#           （模拟雷达点云，供点云转扫描节点消费）
#         - 可选启动 RViz（lidar_to_scan 配置）查看验证结果
# ============================================================================

from launch import LaunchDescription  # launch 框架核心：一组要启动的动作（actions）的容器
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction  # 声明参数、包含其他 launch、定时延迟动作
from launch.conditions import IfCondition  # 条件判断：满足条件时才执行对应动作
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 指明被包含的 launch 文件为 Python 格式
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 参数替换、路径拼接替换
from launch_ros.actions import Node  # ROS2 节点动作：以 ros2 run 方式启动一个节点
from launch_ros.parameter_descriptions import ParameterValue  # 把 launch 参数显式转换为指定 ROS2 参数类型
from launch_ros.substitutions import FindPackageShare  # 运行时查找包的 share 目录


def generate_launch_description():
    # —— launch 参数 ——
    pcd_path = LaunchConfiguration("pcd_path")  # 要回放的 PCD 文件路径
    pcd_frame = LaunchConfiguration("pcd_frame")  # PCD 点云所在坐标系
    publish_hz = LaunchConfiguration("publish_hz")  # 点云发布频率（Hz）
    max_points = LaunchConfiguration("max_points")  # 每帧最多发布的点数
    use_rviz = LaunchConfiguration("use_rviz")  # 是否启动 RViz

    # —— 路径拼接（运行时解析）——
    package_share = FindPackageShare("m20_nav2_system")  # 运行时查找本包的 share 目录
    converter_launch = PathJoinSubstitution([  # 点云转扫描 launch 的路径
        package_share,
        "launch",
        "rslidar_pointcloud_to_scan.launch.py",
    ])
    costmap_params = PathJoinSubstitution([  # costmap 参数文件路径
        package_share,
        "config",
        "nav2_costmap_from_bag.yaml",
    ])
    rviz_config = PathJoinSubstitution([  # RViz 配置文件路径
        package_share,
        "rviz",
        "lidar_to_scan.rviz",
    ])
    default_pcd_path = PathJoinSubstitution([  # 默认回放的 PCD 文件路径
        package_share,
        "maps",
        "pcd", "raw",
        "t100ipro_2026-07-14-11-56-57.pcd",
    ])

    return LaunchDescription([
        # —— 声明参数默认值 ——
        DeclareLaunchArgument("pcd_path", default_value=default_pcd_path),  # 默认 PCD 文件（离线采集数据）
        DeclareLaunchArgument("pcd_frame", default_value="lidar_link"),  # PCD 数据所在坐标系
        DeclareLaunchArgument("publish_hz", default_value="1.0"),  # 默认 1Hz 发布（离线慢速回放）
        DeclareLaunchArgument("max_points", default_value="80000"),  # 每帧最多 8 万点
        DeclareLaunchArgument("use_rviz", default_value="true"),  # 默认启动 RViz 查看结果

        # —— 包含点云转扫描 launch（把 PCD 切片点云转为 /scan）——
        IncludeLaunchDescription(  # 包含 rslidar_pointcloud_to_scan.launch.py
            PythonLaunchDescriptionSource(converter_launch),  # 源文件为 Python 格式的 launch
            launch_arguments={  # 传入参数
                "use_sim_time": "false",  # 离线回放不使用仿真时间
                "cloud_topic": "/LIDAR/POINTS",  # 输入点云话题
                "scan_topic": "/scan",  # 输出扫描话题
                "target_frame": "",  # 不指定目标坐标系（用点云自身坐标系）
            }.items(),
        ),

        # —— 静态 TF 发布 ——
        Node(  # 发布 odom→base_link 静态变换（回放模式无真实里程计，用单位变换占位）
            package="tf2_ros",  # 所属功能包
            executable="static_transform_publisher",  # 静态变换发布可执行文件
            name="static_odom_to_base_link",  # 节点名
            arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],  # 平移/旋转均为 0：odom → base_link
        ),

        Node(  # 发布 base_link→lidar_link 静态变换（雷达在车体上的安装位姿）
            package="tf2_ros",  # 所属功能包
            executable="static_transform_publisher",  # 静态变换发布可执行文件
            name="static_base_link_to_lidar_link",  # 节点名
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "lidar_link"],  # 单位变换：base_link → lidar_link
        ),

        # —— 启动 Nav2 costmap 节点 ——
        Node(  # 启动 nav2_costmap_2d，用 /scan 数据构建代价地图（验证转换结果）
            package="nav2_costmap_2d",  # 所属功能包
            executable="nav2_costmap_2d",  # 可执行文件名
            output="screen",  # 日志输出到屏幕
            parameters=[costmap_params, {"use_sim_time": False}],  # 加载 costmap 参数；离线模式关闭仿真时间
        ),

        # —— 延迟激活 costmap 生命周期 ——
        TimerAction(  # 定时动作：2 秒后激活 costmap
            period=2.0,  # 延迟 2 秒（等 costmap 节点启动完成）
            actions=[
                Node(  # 启动生命周期管理节点，把 costmap 从 unconfigured 推进到 active
                    package="m20_nav2_system",  # 所属功能包
                    executable="lifecycle_configure_activate",  # 生命周期 configure+activate 脚本
                    name="activate_pcd_costmap",  # 节点名
                    output="screen",  # 日志输出到屏幕
                    parameters=[{  # 激活参数
                        "target_node": "/costmap/costmap",  # 目标生命周期节点名
                        "service_timeout_sec": 15.0,  # 服务调用超时 15 秒
                        "transition_timeout_sec": 20.0,  # 状态迁移超时 20 秒
                        "activate_delay_sec": 1.0,  # 激活前额外延迟 1 秒
                    }],
                ),
            ],
        ),

        # —— 延迟启动 PCD 切片发布器 ——
        TimerAction(  # 定时动作：7 秒后开始回放 PCD
            period=7.0,  # 延迟 7 秒（等转换节点与 costmap 全部就绪）
            actions=[
                Node(  # 启动 pcd_slice_publisher：把离线 PCD 按高度切片为 2D 点云并逐帧发布
                    package="m20_nav2_system",  # 所属功能包
                    executable="pcd_slice_publisher",  # 可执行文件名（本包脚本）
                    name="office4f_pcd_slice_publisher",  # 节点名
                    output="screen",  # 日志输出到屏幕
                    parameters=[{  # 回放参数
                        "pcd_path": ParameterValue(pcd_path, value_type=str),  # PCD 文件路径（字符串型）
                        "topic": "/LIDAR/POINTS",  # 发布话题（与转换节点输入一致）
                        "frame_id": ParameterValue(pcd_frame, value_type=str),  # 点云坐标系（字符串型）
                        "publish_hz": ParameterValue(publish_hz, value_type=float),  # 发布频率（浮点型）
                        "min_height": -0.30,  # 切片高度下限（米）
                        "max_height": 0.30,  # 切片高度上限（米）
                        "range_min": 0.20,  # 最小量程（米）
                        "range_max": 30.0,  # 最大量程（米）
                        "max_points": ParameterValue(max_points, value_type=int),  # 每帧最大点数（整型）
                    }],
                ),
            ],
        ),

        # —— 可选启动 RViz ——
        Node(  # 启动 RViz 查看点云/扫描/costmap 验证结果
            condition=IfCondition(use_rviz),  # 仅当 use_rviz=true 时才启动
            package="rviz2",  # 所属功能包
            executable="rviz2",  # 可执行文件名
            name="rviz2_nav2_costmap_from_pcd",  # 节点名（标识为 PCD 验证场景）
            output="screen",  # 日志输出到屏幕
            arguments=["-d", rviz_config],  # 加载指定 RViz 配置文件
            parameters=[{"use_sim_time": False}],  # 离线模式不使用仿真时间
        ),
    ])
