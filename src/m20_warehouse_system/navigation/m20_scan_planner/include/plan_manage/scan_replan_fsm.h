/**
 * @file scan_replan_fsm.h
 * @brief 扫描规划器有限状态机（SCANReplanFSM）头文件
 *
 * 职责：
 *  - 定义规划执行的核心状态机（FSM_EXEC_STATE）与导航模式（NAVI_MODE）；
 *  - 管理规划周期（exec_timer_）与安全检测周期（safety_timer_）两个定时器回调；
 *  - 集成规划管理器（SCANPlannerManager）、可视化（PlanningVisualization）与
 *    各类 ROS2 订阅/发布/服务（目标点、里程计、参考路径、B 样条轨迹、数据展示等）；
 *  - 维护 FSM 运行所需的全部状态量（触发标志、目标/起点/终点状态、路点序列等）。
 *
 * 为 scan_replan_fsm.cpp 的实现声明接口。
 */
#ifndef _SCAN_REPLAN_FSM_H_    // 头文件保护宏：防止重复包含
#define _SCAN_REPLAN_FSM_H_    // 定义保护宏

#include <Eigen/Eigen>                          // Eigen 线性代数库：向量/矩阵/四元数等
#include <algorithm>                            // 标准算法库：std::min/std::max 等
#include <geometry_msgs/msg/pose_stamped.hpp>   // 位姿消息：PoseStamped（RViz 目标点）
#include <iostream>                             // 标准输入输出流：std::cout（状态打印）
#include <m20_warehouse_interfaces/srv/reset_navigation.hpp>   // 自定义服务：ResetNavigation（楼层重置导航）
#include <nav_msgs/msg/odometry.hpp>            // 里程计消息：Odometry（订阅机体位姿）
#include <nav_msgs/msg/path.hpp>                // 路径消息：Path（参考路径/路点输入）
#include <rclcpp/rclcpp.hpp>                    // ROS2 C++ 客户端库：Node/Timer/Subscription 等
#include <std_msgs/msg/bool.hpp>                // 布尔消息：Bool（执行冻结标志）
#include <vector>                               // 标准容器：std::vector
#include <visualization_msgs/msg/marker.hpp>    // 可视化消息：Marker（自膨胀圆柱显示）

#include <bspline_opt/bspline_optimizer.h>      // B 样条优化器：BsplineOptimizer（后端优化）
#include <plan_env/grid_map.h>                  // 栅格地图：GridMap（碰撞检测/膨胀）
#include <scan_planner_msgs/msg/bspline.hpp>    // 自定义消息：Bspline（发布规划结果）
#include <scan_planner_msgs/msg/data_disp.hpp>  // 自定义消息：DataDisp（数据展示）
#include <plan_manage/planner_manager.h>        // 规划管理器：SCANPlannerManager
#include <traj_utils/planning_visualization.h>  // 规划可视化：PlanningVisualization

using std::vector;    // 引入 std::vector 简写

namespace scan_planner    // 扫描规划器命名空间
{

  // 扫描规划器有限状态机：统筹目标接收、轨迹生成/重规划、执行与紧急停车
  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */    // 状态机与模式枚举
    // 状态机执行状态
    enum FSM_EXEC_STATE
    {
      INIT,             // 初始化：等待里程计与触发
      WAIT_TARGET,      // 等待目标
      GEN_NEW_TRAJ,     // 生成新轨迹（首次规划）
      REPLAN_TRAJ,      // 重规划轨迹
      EXEC_TRAJ,        // 执行轨迹
      EMERGENCY_STOP    // 紧急停车
    };
    // 导航目标模式
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,   // 手动目标（RViz 2D 目标点）
      PRESET_TARGET = 2,   // 预设路点（配置中的 fsm.waypoints）
      REFERENCE_PATH = 3,  // 参考路径（initial_path 话题）
    };

    /* planning utils */    // 规划工具对象
    SCANPlannerManager::Ptr planner_manager_;   // 规划管理器（前后端规划算法）
    PlanningVisualization::Ptr visualization_;  // 可视化对象
    scan_planner_msgs::msg::DataDisp data_disp_;   // 数据展示消息（随 FSM 周期发布）

    /* parameters */    // 参数
    int navi_mode_; // 1 manual select, 2 hard code  导航模式：1 手动选择，2 预设路点
    double no_replan_thresh_, replan_thresh_;    // 不重规划/重规划的距离阈值
    std::vector<Eigen::Vector3d> preset_waypoints_;   // 预设路点序列
    int waypoint_num_;                          // 预设路点数量
    double planning_horizon_;                   // 局部规划视界 [m]
    double initial_heading_speed_;              // 新目标起步时沿机体朝向的参考速度
    double startup_replan_lock_sec_;            // 起步阶段禁止普通重规划
    rclcpp::Time trajectory_started_at_;        // 当前轨迹开始时间
    rclcpp::Time last_replan_publish_at_;       // 上次成功发布轨迹时间
    double min_replan_interval_sec_;            // 成功轨迹最小发布间隔
    double replan_retry_cooldown_sec_;          // 失败重规划退避间隔
    rclcpp::Time last_replan_attempt_at_;      // 最近一次重规划尝试时间
    double emergency_time_;                     // 紧急停车判定时间窗 [s]
    double rviz_goal_height_;                   // RViz 目标点高度（取自初始机体 z）
    double self_inflation_z_up_, self_inflation_z_down_;   // 自膨胀圆柱上下延伸量 [m]
    double self_double_cylinder_radius_, self_double_cylinder_offset_;   // 自膨胀双圆柱半径与前后偏移 [m]
    double body_height_;                        // 机体高度（用于参考路径 z 修正）
    std::string self_inflation_frame_id_;       // 自膨胀标记的坐标系

    /* planning data */    // 规划数据
    bool trigger_, have_target_, have_odom_, have_new_target_;   // 触发/有目标/有里程计/有新目标标志
    bool preset_started_{false};                // 预设路点模式是否已启动
    bool rviz_height_ready_;                    // RViz 目标高度是否已就绪
    bool go2_execution_frozen_;                 // 下游执行是否被冻结
    bool enable_fail_safe_, need_hover_stop_;   // 是否启用失效保护 / 是否需要悬停停车
    FSM_EXEC_STATE exec_state_;                 // 当前 FSM 状态
    int continuously_called_times_{0};          // 同一状态连续被调用的次数
    int replan_fail_count_{0};                  // 连续重规划失败计数
    int max_replan_fail_count_{1000};           // 最大重规划失败次数（超限转紧急停车）
    rclcpp::Time last_freeze_update_time_;      // 上次时间冻结更新的时刻

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state  里程计状态：位置/速度/加速度
    Eigen::Quaterniond odom_orient_;            // 里程计姿态四元数

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state  起始状态
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state  目标状态
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state  局部目标状态
    std::vector<Eigen::Vector3d> active_waypoints_;  // 激活的路点序列
    int current_wp_;                            // 当前路点索引

    bool flag_escape_emergency_;                // 紧急停车是否已执行过（避免重复调用）

    /* ROS utils */    // ROS2 通信对象
    rclcpp::Node *node_{nullptr};               // 所属 ROS2 节点指针
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;   // FSM 执行定时器 / 安全检测定时器
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;   // RViz 目标点订阅
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;           // 里程计订阅
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;               // 参考路径订阅
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;   // 执行冻结标志订阅
    rclcpp::Service<m20_warehouse_interfaces::srv::ResetNavigation>::SharedPtr
        reset_navigation_service_;              // 重置导航服务
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;     // B 样条轨迹发布
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;  // 数据展示发布
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;  // 自膨胀标记发布

    /* helper functions */    // 辅助函数
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method  调用反弹重规划（前端+后端）
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method  调用紧急停车
    bool planFromCurrentTraj();                 // 基于当前轨迹状态重新规划
    void setStartStateFromOdomOrCurrentTraj();  // 从里程计或当前轨迹设置起始状态

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    /* 返回值：std::pair< 同一状态连续被调用的次数, 当前状态 > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);   // 切换 FSM 状态并记录调用位置
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();   // 查询连续调用次数与当前状态
    void printFSMExecState();                   // 打印当前 FSM 状态

    void planGlobalTrajbyGivenWps();            // 由预设路点生成全局轨迹
    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);   // 由给定路点生成全局轨迹
    bool planNextWaypoint();                    // 规划到下一个路点
    bool isWaypointSequenceMode() const;        // 是否处于路点序列模式
    bool adjustGlobalTargetIfOccupied();        // 若全局终点被占据则沿轨迹前移调整
    void getLocalTarget();                      // 计算局部目标点（沿全局轨迹按视界推进）
    void finishProcess();                       // 处理重规划失败累计（超限转紧急停车）
    void publishSelfInflationMarker();          // 发布自膨胀双圆柱可视化标记
    double getOdomYaw() const;                  // 从里程计姿态提取偏航角
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;   // 由轨迹段方向估计偏航角
    void updateLocalTrajTimeFreeze();           // 执行冻结时同步推进局部轨迹起始时间（时间冻结）

    /* ROS functions */    // ROS2 回调函数
    void execFSMCallback();                     // FSM 执行定时器回调
    void checkCollisionCallback();              // 安全检测（碰撞检查）定时器回调
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);   // RViz 目标回调
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);   // 路点回调
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);       // 参考路径回调
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);   // 里程计回调
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);   // 执行冻结回调
    void resetNavigationCallback(
        const std::shared_ptr<
            m20_warehouse_interfaces::srv::ResetNavigation::Request> request,   // 重置导航服务请求
        std::shared_ptr<
            m20_warehouse_interfaces::srv::ResetNavigation::Response> response);  // 重置导航服务响应

    bool checkCollision();                      // 检查当前轨迹是否碰撞

  public:
    SCANReplanFSM(/* args */)   // 默认构造函数（无初始化逻辑）
    {
    }
    ~SCANReplanFSM()            // 默认析构函数
    {
    }

    void init(rclcpp::Node *node);   // 初始化：读取参数、创建模块与订阅/发布/服务

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 使类内 Eigen 成员内存 16 字节对齐
  };

} // namespace scan_planner

#endif    // 头文件保护宏结束
