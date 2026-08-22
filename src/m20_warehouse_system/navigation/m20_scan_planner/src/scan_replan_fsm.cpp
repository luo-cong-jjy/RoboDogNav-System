/**
 * @file scan_replan_fsm.cpp
 * @brief 扫描规划器有限状态机实现（SCANReplanFSM）
 *
 * 职责：
 *  - 实现 FSM 主循环（execFSMCallback，10 ms 周期）：管理 INIT / WAIT_TARGET /
 *    GEN_NEW_TRAJ / REPLAN_TRAJ / EXEC_TRAJ / EMERGENCY_STOP 状态流转；
 *  - 实现安全检测（checkCollisionCallback，50 ms 周期）：沿当前局部轨迹采样
 *    检查栅格占据，发现碰撞时尝试重规划或触发紧急停车；
 *  - 处理目标输入（RViz 目标点 / 参考路径 / 预设路点）与里程计、执行冻结、
 *    重置导航服务等 ROS2 回调；
 *  - 计算局部目标（getLocalTarget：沿全局轨迹按规划视界推进，含占据调整）、
 *    调用规划管理器生成/重规划轨迹，并发布 B 样条轨迹与可视化。
 */
// 文件首行为空行（保持原样）

#include <plan_manage/scan_replan_fsm.h>   // 状态机声明（本文件实现其接口）
#include <cmath>        // 数学库：std::atan2 / std::floor / std::ceil 等
#include <stdexcept>    // 异常类型：std::runtime_error（参数校验失败）

namespace    // 匿名命名空间：仅本翻译单元可见的辅助函数
{
  // 读取参数模板：缺失则声明默认值后返回当前值
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);   // 参数未声明则按默认值声明
    return node->get_parameter(name).get_value<T>();   // 读取并返回参数值
  }
} // namespace

namespace scan_planner    // 扫描规划器命名空间
{

  // 状态机初始化：读取 FSM/地图参数、创建规划模块、建立定时器与订阅/发布/服务
  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;                    // 保存节点指针
    current_wp_ = 0;                 // 重置当前路点索引
    exec_state_ = FSM_EXEC_STATE::INIT;   // 初始状态：INIT
    trigger_ = false;                // 重置触发标志
    have_target_ = false;            // 重置目标标志
    have_odom_ = false;              // 重置里程计标志
    have_new_target_ = false;        // 重置新目标标志
    rviz_height_ready_ = false;      // 重置 RViz 高度就绪标志
    go2_execution_frozen_ = false;   // 重置执行冻结标志
    flag_escape_emergency_ = true;   // 允许执行紧急停车（首次进入时调用）
    need_hover_stop_ = false;        // 重置悬停停车需求
    replan_fail_count_ = 0;          // 重置重规划失败计数
    last_freeze_update_time_ = node_->now();   // 初始化时间冻结基准时刻

    /*  fsm param  */    // FSM 参数
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);   // 导航模式（1 手动 / 2 预设 / 3 参考路径）
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);   // 重规划触发距离阈值
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);   // 不重规划距离阈值
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);   // 局部规划视界 [m]
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);   // 紧急停车判定时间窗 [s]
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);   // 是否启用失效保护
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);   // 最大重规划失败次数
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);   // 自膨胀上延伸 [m]
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);   // 自膨胀下延伸 [m]
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);   // 自膨胀双圆柱半径 [m]
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);   // 双圆柱前后偏移 [m]
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);   // 机体高度 [m]
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");   // 自膨胀标记坐标系

    if (navi_mode_ == NAVI_MODE::PRESET_TARGET)   // 预设路点模式：解析路点参数
    {
      const auto flat_waypoints = load_parameter<std::vector<double>>(node_, "fsm.waypoints", {});   // 读取扁平路点数组
      if (flat_waypoints.empty() || flat_waypoints.size() % 3 != 0)   // 路点为空或不是 3 的倍数
        throw std::runtime_error("navi_mode=2 requires non-empty fsm.waypoints with x,y,z triples");   // 参数非法：抛异常
      waypoint_num_ = static_cast<int>(flat_waypoints.size() / 3);   // 路点数量
      preset_waypoints_.resize(waypoint_num_);   // 预分配路点容器
      for (int i = 0; i < waypoint_num_; i++)
      {
        preset_waypoints_[i] = Eigen::Vector3d(flat_waypoints[3 * i], flat_waypoints[3 * i + 1],   // 每 3 个数组成一个路点
                                               flat_waypoints[3 * i + 2]);
      }
    }

    /* initialize main modules */   // 初始化主模块
    visualization_.reset(new PlanningVisualization(node_));   // 创建可视化对象
    planner_manager_.reset(new SCANPlannerManager);           // 创建规划管理器
    planner_manager_->initPlanModules(node_, visualization_); // 初始化规划模块（读参数/建地图/建优化器）

    /* callback */    // 创建定时器与回调
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),   // FSM 执行定时器：100 Hz
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),   // 安全检测定时器：20 Hz
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(   // 订阅机体里程计（传感器 QoS）
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(   // 订阅执行冻结标志
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    reset_navigation_service_ =                // 创建重置导航服务（楼层更换时调用）
        node_->create_service<m20_warehouse_interfaces::srv::ResetNavigation>(
            "reset",
            std::bind(
                &SCANReplanFSM::resetNavigationCallback,
                this,
                std::placeholders::_1,
                std::placeholders::_2));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);   // 发布 B 样条轨迹
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);   // 发布数据展示
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(   // 发布自膨胀标记（transient_local）
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)   // 手动模式：订阅 RViz 目标点
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)   // 参考路径模式：订阅初始路径
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::PRESET_TARGET)   // 预设模式：等待首个里程计后自动启动
      RCLCPP_INFO(node_->get_logger(), "Preset waypoint mode will start after the first odometry message");
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");   // 模式非法：抛异常
  }

  // 由预设路点启动规划：显示路点、触发并规划到第一个路点
  void SCANReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps = preset_waypoints_;   // 拷贝预设路点

    for (size_t i = 0; i < wps.size(); i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);   // 逐个显示目标点
    }

    active_waypoints_ = wps;             // 激活路点序列
    current_wp_ = 0;                     // 从第一个路点开始
    trigger_ = true;                     // 置触发标志
    init_pt_ = odom_pos_;                // 记录初始位置

    if (planNextWaypoint())              // 成功规划到第一个路点
    {
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");   // 转入生成新轨迹状态
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory to first preset waypoint");   // 规划失败打印错误
    }
  }

  // RViz 目标点回调：收到 2D 目标后包装成单点路径交给 waypointCallback
  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)                    // 空消息
      return;

    if (!rviz_height_ready_)     // 尚未收到初始机体位姿（高度未就绪）
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");   // 忽略目标
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();   // 构造单点路径
    path->header = msg->header;            // 复用目标点头
    path->poses.push_back(*msg);           // 把目标点作为唯一 pose
    waypointCallback(path);                // 复用路点处理流程
  }

  // 路点回调：以里程计为起点规划到第一个路点（即目标点），成功后触发 FSM
  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())        // 空路径
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,   // 节流打印警告
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)   // 目标 z 异常（< -0.1）：忽略
      return;

    cout << "Triggered!" << endl;          // 打印触发
    trigger_ = true;                       // 置触发标志
    init_pt_ = odom_pos_;                  // 记录初始位置

    bool success = false;                  // 规划成功标志
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;   // 目标点（z 用 RViz 高度）
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());   // 生成全局轨迹

    if (success)
      success = adjustGlobalTargetIfOccupied();   // 目标被占据则调整

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);   // 显示目标点

    if (success)
    {

      /*** display ***/    // 显示全局轨迹
      constexpr double step_size_t = 0.1;   // 轨迹显示采样步长 [s]
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);   // 采样点数
      vector<Eigen::Vector3d> gloabl_traj(i_end);   // 采样点容器
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);   // 逐点求值
      }

      end_vel_.setZero();                  // 终点速度置 0
      have_target_ = true;                 // 置有目标标志
      have_new_target_ = true;             // 置有新目标标志

      /*** FSM ***/    // 按当前状态流转
      if (exec_state_ == WAIT_TARGET)      // 等待目标中：转入生成新轨迹
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ)   // 执行中：转入重规划
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);   // 显示全局路径
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");   // 规划失败打印错误
    }
  }

  // 由给定路点生成全局轨迹（参考路径模式的核心实现）
  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())                 // 路点为空
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");   // 打印警告
      return false;
    }

    end_pt_ = waypoints.back();            // 目标点取最后一个路点

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);   // 逐个显示路点
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(   // 调用规划管理器生成全局轨迹
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)                          // 生成失败
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");   // 打印错误
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())   // 终点被占据且无法调整
      return false;

    constexpr double step_size_t = 0.1;    // 显示采样步长 [s]
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);   // 采样点数
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);   // 采样点容器
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);   // 逐点求值
    }

    end_vel_.setZero();                    // 终点速度置 0
    have_target_ = true;                   // 置有目标标志
    have_new_target_ = true;               // 置有新目标标志
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);   // 显示全局路径
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);   // 显示最终目标

    return true;
  }

  // 规划到下一个路点（预设路点模式）
  bool SCANReplanFSM::planNextWaypoint()
  {
    if (current_wp_ < 0 || current_wp_ >= (int)active_waypoints_.size())   // 路点索引越界
    {
      RCLCPP_WARN(node_->get_logger(), "[navi_mode=%d] No active waypoint to plan", navi_mode_);   // 打印警告
      return false;
    }

    end_pt_ = active_waypoints_[current_wp_];   // 目标点 = 当前路点
    setStartStateFromOdomOrCurrentTraj();       // 设置起始状态（里程计或当前轨迹）

    bool success = planner_manager_->planGlobalTraj(   // 生成到当前路点的全局轨迹
        start_pt_,
        start_vel_,
        start_acc_,
        end_pt_,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)                          // 生成失败
    {
      RCLCPP_ERROR(node_->get_logger(), "[navi_mode=%d] Unable to generate trajectory to waypoint %d",   // 打印错误
                   navi_mode_, current_wp_ + 1);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())   // 终点被占据且无法调整
      return false;

    constexpr double step_size_t = 0.1;    // 显示采样步长 [s]
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);   // 采样点数
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);   // 采样点容器
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);   // 逐点求值
    }

    end_vel_.setZero();                    // 终点速度置 0
    have_target_ = true;                   // 置有目标标志
    have_new_target_ = true;               // 置有新目标标志
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);   // 显示全局路径
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, current_wp_);   // 显示当前路点
    RCLCPP_INFO(node_->get_logger(), "[navi_mode=%d] Planning to waypoint %d/%zu: [%.2f, %.2f, %.2f]",   // 打印规划信息
                navi_mode_, current_wp_ + 1, active_waypoints_.size(), end_pt_(0), end_pt_(1), end_pt_(2));

    return true;
  }

  // 是否处于路点序列模式（预设路点）
  bool SCANReplanFSM::isWaypointSequenceMode() const
  {
    return navi_mode_ == NAVI_MODE::PRESET_TARGET;   // 预设模式即路点序列模式
  }

  // 若全局轨迹终点被占据：从终点沿轨迹向起点搜索最近的自由点作为新终点
  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;            // 栅格地图
    auto &global_data = planner_manager_->global_data_;   // 全局轨迹数据
    const double duration = global_data.global_duration_;   // 全局轨迹时长
    if (!map || duration < 1e-3)         // 地图无效或轨迹过短
      return true;

    constexpr double sample_dt = 0.05;   // 占据采样步长 [s]
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));   // 采样点数
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);   // 终点位置
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);   // 终点前一点
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));   // 终点占据值（含膨胀）
    if (final_occ <= 0)                  // 终点自由
      return true;

    for (int i = sample_num; i >= 0; --i)   // 从终点向起点倒序搜索
    {
      const double t = duration * i / sample_num;   // 采样时刻
      const double prev_t = duration * std::max(0, i - 1) / sample_num;   // 前一点时刻
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);    // 采样位置
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);   // 前一点位置

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)   // 找到自由点
      {
        const Eigen::Vector3d raw_end = end_pt_;   // 保存原终点
        end_pt_ = pt;                    // 更新终点为该自由点
        global_data.global_duration_ = t;          // 截断全局轨迹时长
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);   // 更新进度时间
        RCLCPP_WARN(node_->get_logger(),   // 打印调整信息
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),    // 未找到自由点：打印错误
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  // 参考路径回调：解析路径点（z 加机体高度），生成全局轨迹并触发 FSM
  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())      // 空路径
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,   // 节流打印警告
                           "Received empty initial_path; ignoring");
      return;
    }

    trigger_ = true;                     // 置触发标志

    std::vector<Eigen::Vector3d> waypoints;   // 路点容器
    waypoints.reserve(msg->poses.size());     // 预分配容量

    for (const auto& pose_stamped : msg->poses)   // 遍历路径点
    {
      Eigen::Vector3d wp;                // 单个路点
      wp(0) = pose_stamped.pose.position.x;      // x
      wp(1) = pose_stamped.pose.position.y;      // y
      wp(2) = pose_stamped.pose.position.z + body_height_; // Adjust for body height   z 加机体高度（路径是地面高度）
      waypoints.push_back(wp);           // 加入路点序列
    }
    bool success = planGlobalTrajByWaypoints(waypoints);   // 由路点生成全局轨迹

    if (success)
    {
      /*** FSM ***/    // 按当前状态流转
      if (exec_state_ == WAIT_TARGET)    // 等待目标中
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");   // 转入生成新轨迹
      }
      else if (exec_state_ == EXEC_TRAJ) // 执行中
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");    // 转入重规划
      }

      RCLCPP_INFO(node_->get_logger(), "Reference path accepted");   // 打印接受信息
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from reference path");   // 打印错误
    }
  }

  // 里程计回调：缓存位置/速度/姿态，设置 RViz 目标高度，触发预设模式启动
  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;   // 位置 x
    odom_pos_(1) = msg->pose.pose.position.y;   // 位置 y
    odom_pos_(2) = msg->pose.pose.position.z;   // 位置 z

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)   // 手动模式且高度未就绪
    {
      rviz_goal_height_ = odom_pos_(2);          // 用初始机体 z 作为 RViz 目标高度
      rviz_height_ready_ = true;                 // 置就绪标志
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);   // 打印
    }

    odom_vel_(0) = msg->twist.twist.linear.x;    // 速度 x
    odom_vel_(1) = msg->twist.twist.linear.y;    // 速度 y
    odom_vel_(2) = msg->twist.twist.linear.z;    // 速度 z

    //odom_acc_ = estimateAcc( msg );   // 加速度估计（暂未启用）

    odom_orient_.w() = msg->pose.pose.orientation.w;   // 姿态四元数 w
    odom_orient_.x() = msg->pose.pose.orientation.x;   // 四元数 x
    odom_orient_.y() = msg->pose.pose.orientation.y;   // 四元数 y
    odom_orient_.z() = msg->pose.pose.orientation.z;   // 四元数 z

    have_odom_ = true;                   // 置里程计标志
    publishSelfInflationMarker();        // 发布自膨胀可视化标记
    if (navi_mode_ == NAVI_MODE::PRESET_TARGET && !preset_started_)   // 预设模式且未启动
    {
      preset_started_ = true;            // 置启动标志
      planGlobalTrajbyGivenWps();        // 由预设路点启动规划
    }
  }

  // 执行冻结回调：记录下游执行冻结状态
  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;   // 更新冻结标志
  }

  // 重置导航服务回调：楼层更换后清空地图/轨迹/目标并回到 WAIT_TARGET
  void SCANReplanFSM::resetNavigationCallback(
      const std::shared_ptr<
          m20_warehouse_interfaces::srv::ResetNavigation::Request> request,   // 服务请求（floor_id / generation）
      std::shared_ptr<
          m20_warehouse_interfaces::srv::ResetNavigation::Response> response)  // 服务响应
  {
    // Floor replacement invalidates the map, target, and every trajectory
    // derived from the previous PCD. Publish a stationary trajectory first.
    // 楼层更换会使地图、目标以及所有由旧 PCD 派生的轨迹失效：先发布静止轨迹。
    if (have_odom_)                      // 有里程计
      callEmergencyStop(odom_pos_);      // 发布原地停止轨迹

    if (planner_manager_ && planner_manager_->grid_map_)   // 重置栅格地图缓冲区
      planner_manager_->grid_map_->resetBuffer();

    active_waypoints_.clear();           // 清空激活路点
    current_wp_ = 0;                     // 重置路点索引
    trigger_ = false;                    // 重置触发
    have_target_ = false;                // 重置目标
    have_new_target_ = false;            // 重置新目标
    replan_fail_count_ = 0;              // 重置失败计数
    need_hover_stop_ = false;            // 重置悬停停车
    flag_escape_emergency_ = true;       // 允许紧急停车
    continuously_called_times_ = 0;      // 重置连续调用计数

    if (planner_manager_)                // 重置局部轨迹数据
    {
      auto &local = planner_manager_->local_data_;
      local.duration_ = 0.0;             // 轨迹时长置 0
      local.start_time_ = rclcpp::Time(  // 起始时间置 0
          0, 0, node_->get_clock()->get_clock_type());

      auto &global = planner_manager_->global_data_;   // 重置全局轨迹数据
      global.local_traj_.clear();        // 清空局部轨迹
      global.global_duration_ = 0.0;     // 全局时长置 0
      global.local_start_time_ = -1.0;   // 局部起止时间置 -1
      global.local_end_time_ = -1.0;
      global.time_increase_ = 0.0;       // 时间增量置 0
      global.last_time_inc_ = 0.0;
      global.last_progress_time_ = 0.0;  // 进度时间置 0
    }

    last_freeze_update_time_ = node_->now();   // 更新时间冻结基准
    changeFSMExecState(WAIT_TARGET, "FLOOR_RESET");   // 回到等待目标状态
    response->success = true;            // 服务响应：成功
    response->message =                  // 响应消息（含楼层与代次）
        "navigation reset for floor=" + request->floor_id +
        ", generation=" + std::to_string(request->generation);
    RCLCPP_INFO(node_->get_logger(), "%s", response->message.c_str());   // 打印响应消息
  }

  // 时间冻结更新：下游执行冻结时同步推进局部轨迹起始时间（冻结轨迹时钟）
  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();   // 当前时刻
    double dt = (now - last_freeze_update_time_).seconds();   // 距上次更新的时间
    last_freeze_update_time_ = now;      // 更新基准时刻

    if (dt <= 0.0 || dt > 0.2)           // 时间差非法/过大
      return;

    LocalTrajData *info = &planner_manager_->local_data_;   // 局部轨迹数据
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)   // 执行冻结且轨迹已开始
      info->start_time_ += rclcpp::Duration::from_seconds(dt);   // 起始时间后移（等效冻结轨迹时钟）
  }

  // 从里程计姿态提取偏航角（机体 x 轴在世界系的方向角）
  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);   // 机体 x 轴（世界系）
    if (heading.head<2>().squaredNorm() < 1e-8)   // 方向退化
      return 0.0;                    // 返回 0
    return std::atan2(heading(1), heading(0));    // 反正切求偏航角
  }

  // 由轨迹段方向（from->to）估计偏航角；段过短则退回机体当前偏航
  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));   // 平面方向向量
    if (diff.squaredNorm() < 1e-8)     // 段过短
      return getOdomYaw();             // 退回机体偏航
    return std::atan2(diff(1), diff(0));   // 反正切求方向角
  }

  // 发布自膨胀双圆柱可视化标记（前后两个圆柱，含 z 上下延伸）
  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);   // 圆柱半径（非负）
    const double z_up = std::max(0.0, self_inflation_z_up_);             // 上延伸（非负）
    const double z_down = std::max(0.0, self_inflation_z_down_);         // 下延伸（非负）
    const double height = std::max(1e-3, z_up + z_down);                 // 圆柱高度

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;   // 坐标系
    marker.header.stamp = node_->now();   // 时间戳
    marker.ns = "self_inflation";         // 命名空间
    marker.type = visualization_msgs::msg::Marker::CYLINDER;   // 圆柱类型
    marker.action = visualization_msgs::msg::Marker::ADD;      // 添加操作
    marker.pose.orientation.w = 1.0;      // 默认姿态（无旋转）
    marker.scale.x = 2.0 * radius;        // 直径 x
    marker.scale.y = 2.0 * radius;        // 直径 y
    marker.scale.z = height;              // 高度
    marker.color.r = 0.1;                 // 颜色 R
    marker.color.g = 0.6;                 // 颜色 G
    marker.color.b = 1.0;                 // 颜色 B
    marker.color.a = 0.4;                 // 透明度
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);   // 生存期（短暂显示）

    Eigen::Vector3d center = odom_pos_;   // 中心 = 机体位置
    center(2) += 0.5 * (z_up - z_down);   // z 中心偏移（上下延伸不对称时的中心）

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);   // 机体朝向单位向量
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;   // 前圆柱位置
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;    // 后圆柱位置

    marker.id = 0;                        // 前圆柱 ID
    marker.pose.position.x = front(0);    // 位置 x
    marker.pose.position.y = front(1);    // 位置 y
    marker.pose.position.z = front(2);    // 位置 z
    self_inflation_pub_->publish(marker); // 发布前圆柱

    marker.id = 1;                        // 后圆柱 ID
    marker.pose.position.x = rear(0);     // 位置 x
    marker.pose.position.y = rear(1);     // 位置 y
    marker.pose.position.z = rear(2);     // 位置 z
    self_inflation_pub_->publish(marker); // 发布后圆柱
  }

  // 切换 FSM 状态：维护连续调用计数并打印状态迁移
  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)         // 状态未变化
      continuously_called_times_++;       // 连续调用计数 +1
    else
      continuously_called_times_ = 1;     // 状态变化：重置计数

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};   // 状态名映射
    int pre_s = int(exec_state_);         // 旧状态索引
    exec_state_ = new_state;              // 更新状态
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;   // 打印状态迁移
  }

  // 返回同一状态连续被调用的次数与当前状态
  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);   // 组合返回
  }

  // 打印当前 FSM 状态
  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};   // 状态名映射

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;   // 打印当前状态
  }

  // FSM 执行主回调（100 Hz）：状态机流转 + 定时打印状态
  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();         // 先处理时间冻结（执行冻结时冻结轨迹时钟）

    static int fsm_num = 0;              // 状态打印计数（静态）
    fsm_num++;                           // 递增
    if (fsm_num == 100)                  // 每 100 个周期（约 1 s）打印一次状态
    {
      printFSMExecState();               // 打印状态
      if (!have_odom_)                   // 无里程计
        cout << "no odom." << endl;      // 打印提示
      if (!trigger_)                     // 未触发
        cout << "wait for goal." << endl;   // 打印等待目标
      fsm_num = 0;                       // 重置计数
    }

    switch (exec_state_)                 // 按当前状态分派
    {
    case INIT:                           // 初始化状态
    {
      if (!have_odom_)                   // 无里程计：等待
      {
        return;
      }
      if (!trigger_)                     // 未触发：等待
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");   // 有里程计且有目标：转入等待目标
      break;
    }

    case WAIT_TARGET:                    // 等待目标状态
    {
      if (!have_target_)                 // 尚无目标：等待
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");   // 有目标：转入生成新轨迹
      }
      break;
    }

    case GEN_NEW_TRAJ:                   // 生成新轨迹状态
    {
      setStartStateFromOdomOrCurrentTraj();   // 设置起始状态

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;        // 是否使用随机多项式初始路径
      if (timesOfConsecutiveStateCalls().first == 1)   // 首次连续调用：用确定性初始路径
        flag_random_poly_init = false;
      else                               // 连续失败重试：改用随机初始路径（增加多样性）
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);   // 调用反弹重规划
      if (success)                       // 规划成功
      {

        replan_fail_count_ = 0;          // 重置失败计数
        changeFSMExecState(EXEC_TRAJ, "FSM");   // 转入执行轨迹
        flag_escape_emergency_ = true;   // 允许紧急停车
      }
      else                               // 规划失败
      {
        replan_fail_count_++;            // 失败计数 +1
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");   // 留在生成新轨迹（下一周期重试）
      }
      break;
    }

    case REPLAN_TRAJ:                    // 重规划状态
    {
      if (planFromCurrentTraj())         // 基于当前轨迹重规划成功
      {
        replan_fail_count_ = 0;          // 重置失败计数
        changeFSMExecState(EXEC_TRAJ, "FSM");   // 转入执行轨迹
      }
      else                               // 重规划失败
      {
        replan_fail_count_++;            // 失败计数 +1
        changeFSMExecState(REPLAN_TRAJ, "FSM");   // 留在重规划（下一周期重试）
      }

      break;
    }

    case EXEC_TRAJ:                      // 执行轨迹状态
    {
      /* determine if need to replan */  // 判断是否需要重规划
      LocalTrajData *info = &planner_manager_->local_data_;   // 局部轨迹数据
      rclcpp::Time time_now = node_->now();   // 当前时刻
      double t_cur = (time_now - info->start_time_).seconds();   // 当前轨迹时刻
      t_cur = min(info->duration_, t_cur);   // 截断到轨迹末

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);   // 当前期望位置

      if (isWaypointSequenceMode() &&    // 路点序列模式：接近当前路点时提前切到下一个路点
          current_wp_ + 1 < (int)active_waypoints_.size() &&
          (end_pt_ - odom_pos_).norm() < 0.5)
      {
        current_wp_++;                   // 切换到下一个路点
        if (planNextWaypoint())          // 成功规划下一路点
        {
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");   // 转入生成新轨迹
          return;
        }
        replan_fail_count_++;            // 失败计数 +1
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");   // 转入生成新轨迹（重试）
        return;
      }

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)   // 轨迹已接近结束
      {
        if (isWaypointSequenceMode() && current_wp_ + 1 < (int)active_waypoints_.size())   // 还有下一个路点
        {
          current_wp_++;                 // 切换到下一个路点
          if (planNextWaypoint())        // 成功规划下一路点
          {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");   // 转入生成新轨迹
            return;
          }
          replan_fail_count_++;          // 失败计数 +1
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");   // 转入生成新轨迹（重试）
          return;
        }

        if (isWaypointSequenceMode())    // 路点序列全部完成
        {
          active_waypoints_.clear();     // 清空路点
          current_wp_ = 0;               // 重置路点索引
        }

        have_target_ = false;            // 清除目标标志

        changeFSMExecState(WAIT_TARGET, "FSM");   // 回到等待目标
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)   // 距终点很近：无需重规划
      {
        // cout << "near end" << endl;
        return;                          // 直接返回
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)   // 距起点很近：无需重规划
      {
        // cout << "near start" << endl;
        return;                          // 直接返回
      }
      else                               // 距起终点都较远：需要重规划
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");   // 转入重规划
      }
      break;
    }

    case EMERGENCY_STOP:                 // 紧急停车状态
    {

      if (flag_escape_emergency_) // Avoiding repeated calls  避免重复调用
      {
        callEmergencyStop(odom_pos_);    // 执行紧急停车（原地停止轨迹）
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)   // 失效保护开启、非悬停模式且已停下
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");   // 重新规划继续
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)   // 悬停模式且已停下
        {
          RCLCPP_INFO(node_->get_logger(),   // 打印退出信息
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;      // 清除悬停需求
          have_target_ = false;          // 清除目标
          trigger_ = false;              // 清除触发
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");   // 回到等待目标
        }
      }

      flag_escape_emergency_ = false;    // 标记紧急停车已执行
      break;
    }
    }

    finishProcess();                     // 处理重规划失败累计（超限转紧急停车）

    data_disp_.header.stamp = node_->now();   // 更新时间戳
    data_disp_pub_->publish(data_disp_);      // 发布数据展示消息
  }

  // 处理重规划失败累计：超过最大次数则转入紧急停车
  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)   // 失败次数超限
    {
      RCLCPP_WARN(node_->get_logger(),   // 打印警告
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;            // 重置失败计数
      need_hover_stop_ = true;           // 置悬停停车需求
      flag_escape_emergency_ = true;     // 允许紧急停车
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");   // 转入紧急停车
    }
  }

  // 基于当前轨迹重规划：从当前轨迹取起始状态，重新生成全局轨迹并反弹重规划
  bool SCANReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;   // 局部轨迹数据
    rclcpp::Time time_now = node_->now();   // 当前时刻
    double t_cur = (time_now - info->start_time_).seconds();   // 当前轨迹时刻
    t_cur = std::min(std::max(t_cur, 0.0), info->duration_);   // 截断到 [0, duration]

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = odom_pos_;               // 起点 = 当前机体位置
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);   // 起点速度 = 当前轨迹速度
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);   // 起点加速度 = 当前轨迹加速度

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();   // 指向目标的平面向量
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)   // 当前速度背向目标
    {
      start_vel_.setZero();              // 速度清零（避免倒车远离目标）
      start_acc_.setZero();              // 加速度清零
    }

    if (!planner_manager_->planGlobalTraj(   // 重新生成全局轨迹
            start_pt_,
            start_vel_,
            start_acc_,
            end_pt_,
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero()))
    {
      RCLCPP_ERROR(node_->get_logger(),  // 打印错误
                   "[navi_mode=%d] Unable to refresh global trajectory from odom to current target", navi_mode_);
      return false;
    }

    if (!adjustGlobalTargetIfOccupied()) // 终点被占据且无法调整
      return false;

    bool success = callReboundReplan(true, false);   // 反弹重规划（确定性初始路径）
    if (!success)                        // 失败则用随机初始路径再试一次
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;                         // 重规划成功
  }

  // 设置起始状态：优先取当前轨迹的状态（时间戳有效时），否则用里程计状态
  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;               // 起点 = 机体位置
    start_vel_ = odom_vel_;              // 速度 = 里程计速度
    start_acc_.setZero();                // 加速度置 0

    LocalTrajData *info = &planner_manager_->local_data_;   // 局部轨迹数据
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)   // 轨迹未开始或为空
      return;                            // 用里程计状态

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();   // 当前轨迹时刻（原始）
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)   // 时刻超出有效范围
      return;                            // 用里程计状态

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);   // 截断到轨迹范围
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);   // 起点速度 = 当前轨迹速度
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);   // 起点加速度 = 当前轨迹加速度

    const Eigen::Vector2d to_goal = end_pt_.head<2>() - odom_pos_.head<2>();   // 指向目标的平面向量
    if (to_goal.norm() > 1e-3 && start_vel_.head<2>().dot(to_goal) < 0.0)   // 当前速度背向目标
    {
      start_vel_.setZero();              // 速度清零
      start_acc_.setZero();              // 加速度清零
    }
  }

  // 安全检测回调（20 Hz）：沿当前轨迹采样检查栅格占据，发现碰撞则重规划或紧急停车
  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();         // 先处理时间冻结

    LocalTrajData *info = &planner_manager_->local_data_;   // 局部轨迹数据
    auto map = planner_manager_->grid_map_;   // 栅格地图

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)   // 无执行轨迹
      return;

    /* ---------- check trajectory ---------- */   // 检查轨迹碰撞
    constexpr double time_step = 0.01;   // 检查采样步长 [s]
    double t_cur = (node_->now() - info->start_time_).seconds();   // 当前轨迹时刻
    double t_2_3 = info->duration_ * 2 / 3;   // 轨迹前 2/3 的边界时刻
    for (double t = t_cur; t < info->duration_; t += time_step)   // 沿轨迹逐点检查
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
      // 若当前时刻在前 2/3 内，则只检查轨迹前 2/3 部分（后 1/3 留给重规划，避免误报）
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);   // 当前点位置
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));   // 下一个检查点位置
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))   // 该点处于膨胀占据区
      {
        if (planFromCurrentTraj()) // Make a chance  尝试重规划
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");   // 重规划成功：继续执行
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time  碰撞点距当前时刻不足紧急时间窗
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);   // 打印警告
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");   // 立即紧急停车
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");   // 时间充裕：转入重规划
          }
          return;
        }
        break;
      }
    }
  }

  // 调用反弹重规划并发布结果：计算局部目标、执行规划、发布 B 样条轨迹与可视化
  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();                    // 计算局部目标

    bool plan_success =                  // 调用规划管理器反弹重规划
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;            // 清除新目标标志

    cout << "final_plan_success=" << plan_success << endl;   // 打印规划结果

    if (plan_success)                    // 规划成功：发布轨迹
    {

      auto info = &planner_manager_->local_data_;   // 局部轨迹数据

      /* publish traj */    // 发布轨迹消息
      scan_planner_msgs::msg::Bspline bspline;   // 构造 B 样条消息
      bspline.order = 3;                 // 阶数 = 3（三次样条）
      bspline.start_time = info->start_time_;   // 起始时刻
      bspline.traj_id = info->traj_id_;  // 轨迹 ID

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();   // 位置控制点矩阵
      bspline.pos_pts.reserve(pos_pts.cols());   // 预分配容量
      for (int i = 0; i < pos_pts.cols(); ++i)   // 逐列转成 Point 消息
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);            // x
        pt.y = pos_pts(1, i);            // y
        pt.z = pos_pts(2, i);            // z
        bspline.pos_pts.push_back(pt);   // 加入控制点列表
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();   // 节点向量
      bspline.knots.reserve(knots.rows());   // 预分配容量
      for (int i = 0; i < knots.rows(); ++i)   // 逐元素拷贝节点
      {
        bspline.knots.push_back(knots(i));
      }

      bspline_pub_->publish(bspline);    // 发布 B 样条轨迹

      visualization_->displayOptimalTraj(info->position_traj_, 0);   // 显示最优轨迹
    }

    return plan_success;                 // 返回规划结果
  }

  // 调用紧急停车：生成原地停止轨迹并发布（供执行层停止运动）
  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);   // 生成原地停止轨迹

    auto info = &planner_manager_->local_data_;   // 局部轨迹数据

    /* publish traj */    // 发布轨迹消息
    scan_planner_msgs::msg::Bspline bspline;   // 构造 B 样条消息
    bspline.order = 3;                 // 阶数 = 3
    bspline.start_time = info->start_time_;   // 起始时刻
    bspline.traj_id = info->traj_id_;  // 轨迹 ID

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();   // 位置控制点矩阵
    bspline.pos_pts.reserve(pos_pts.cols());   // 预分配容量
    for (int i = 0; i < pos_pts.cols(); ++i)   // 逐列转成 Point 消息
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);            // x
      pt.y = pos_pts(1, i);            // y
      pt.z = pos_pts(2, i);            // z
      bspline.pos_pts.push_back(pt);   // 加入控制点列表
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();   // 节点向量
    bspline.knots.reserve(knots.rows());   // 预分配容量
    for (int i = 0; i < knots.rows(); ++i)   // 逐元素拷贝节点
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);    // 发布 B 样条轨迹

    return true;                       // 返回成功
  }

  // 计算局部目标：沿全局轨迹按规划视界推进，目标被占据则就近搜索自由点
  void SCANReplanFSM::getLocalTarget()
  {
    double t;                          // 采样时刻

    double t_step = planning_horizon_ / 20 / planner_manager_->pp_.max_vel_;   // 全局轨迹采样步长 [s]
    double dist_min = 9999, dist_min_t = 0.0;   // 最近距离及其时刻（用于进度更新）
    double target_t = planner_manager_->global_data_.global_duration_;   // 目标时刻（默认轨迹末）
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)   // 从进度点沿全局轨迹推进
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);   // 采样位置
      double dist = (pos_t - start_pt_).norm();   // 到起始点的距离

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)   // 起点处距离已超视界（异常）
      {
        RCLCPP_ERROR(node_->get_logger(),   // 打印错误
                     "Local target progress mismatch: distance=%.3f horizon=%.3f progress_time=%.3f",
                     dist, planning_horizon_, planner_manager_->global_data_.last_progress_time_);
        local_target_pt_ = pos_t;      // 取当前点为局部目标
        target_t = t;                  // 记录目标时刻
        planner_manager_->global_data_.last_progress_time_ = t;   // 更新进度时间
        break;
      }
      if (dist < dist_min)             // 更新最近点
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizon_)   // 距离达到视界：取该点为局部目标
      {
        local_target_pt_ = pos_t;      // 局部目标位置
        target_t = t;                  // 目标时刻
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;   // 更新进度时间（最近点时刻）
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // Last global point  已遍历到全局轨迹末
    {
      local_target_pt_ = end_pt_;      // 局部目标 = 最终目标
      target_t = planner_manager_->global_data_.global_duration_;   // 目标时刻 = 轨迹末
    }

    auto targetOccupancy = [&](const Eigen::Vector3d &pt) {   // 局部目标占据检查 lambda
      return planner_manager_->grid_map_->getInflateOccupancy(pt, estimateYawFromSegment(odom_pos_, pt));
    };

    if (targetOccupancy(local_target_pt_) != 0)   // 局部目标处于占据区
    {
      bool found_free_target = false;  // 是否找到自由目标
      double adjusted_t = target_t;    // 调整后的目标时刻

      for (double dt = 0.0; dt <= planner_manager_->global_data_.global_duration_; dt += t_step)   // 以目标时刻为中心向两侧搜索
      {
        double t_forward = target_t + dt;   // 向前搜索
        if (t_forward <= planner_manager_->global_data_.global_duration_)   // 未超出轨迹末
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_forward);   // 向前采样位置
          if (targetOccupancy(pt) == 0)   // 找到自由点
          {
            local_target_pt_ = pt;     // 更新局部目标
            adjusted_t = t_forward;    // 更新目标时刻
            found_free_target = true;  // 标记找到
            break;
          }
        }

        double t_backward = target_t - dt;   // 向后搜索
        if (t_backward >= std::max(0.0, dist_min_t))   // 不早于最近点时刻
        {
          Eigen::Vector3d pt = planner_manager_->global_data_.getPosition(t_backward);   // 向后采样位置
          if (targetOccupancy(pt) == 0)   // 找到自由点
          {
            local_target_pt_ = pt;     // 更新局部目标
            adjusted_t = t_backward;   // 更新目标时刻
            found_free_target = true;  // 标记找到
            break;
          }
        }
      }

      if (found_free_target)           // 找到自由目标
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,   // 节流打印提示
                             "Local target was adjusted to a nearby collision-free point");
        target_t = adjusted_t;         // 更新目标时刻
      }
      else                             // 未找到自由目标
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,   // 节流打印提示
                             "Local target is in collision and no nearby free target was found");
      }
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))   // 距最终目标可在一段减速内到达（v²/2a）
    {
      // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
      // cout << "A" << endl;
      local_target_vel_ = Eigen::Vector3d::Zero();   // 局部目标速度置 0（准备停车）
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(target_t);   // 局部目标速度 = 全局轨迹速度
      // cout << "AA" << endl;
    }
  }

} // namespace scan_planner
