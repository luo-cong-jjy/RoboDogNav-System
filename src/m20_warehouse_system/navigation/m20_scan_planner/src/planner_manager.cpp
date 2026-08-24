/**
 * @file planner_manager.cpp
 * @brief 扫描规划管理器实现（SCANPlannerManager）
 *
 * 职责：
 *  - 实现前端初始化：从多项式初始轨迹（min-snap）或当前轨迹采样出初始路径点，
 *    并调用 A* 对控制点做无碰撞调整（initControlPoints）；
 *  - 实现后端优化：调用 B 样条优化器做反弹重规划（BsplineOptimizeTrajRebound）
 *    与精化（BsplineOptimizeTrajRefine）；
 *  - 实现全局轨迹规划（单目标/路点序列，min-snap 多项式轨迹，过远自动插值中间点）；
 *  - 实现时间重分配（reparamBspline：按比例拉长时间降低速度/加速度）
 *    与动态可行性检查（checkDynamicFeasibility：速度/加速度逐点采样校验）；
 *  - 实现紧急停车（EmergencyStop：生成原地停止轨迹）。
 */
// #include <fstream>
#include <plan_manage/planner_manager.h>   // 规划管理器声明（本文件实现其接口）
#include <plan_env/grid_map.h>
#include "m20_trajectory/scan_optimizer_adapter.h"
#include "m20_trajectory/scan_map_adapter.h"
#include <bspline_opt/uniform_bspline.h>
#include <chrono>        // 时间库：std::chrono（各阶段耗时统计）
#include <thread>        // 线程库（预留）

namespace scan_planner    // 扫描规划器命名空间
{
  namespace    // 匿名命名空间：仅本翻译单元可见的辅助函数
  {
    // 沿路径点按 xy 弧长比例线性插值 z 参考值（从 start_z 渐变到 target_z）
    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())      // 空路径直接返回
        return;

      if (points.size() == 1)  // 只有一个点：直接赋起始 z
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);   // 各点的累计 xy 弧长
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +      // 累加相邻点 xy 距离
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();     // 总 xy 弧长
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6                    // 归一化比例：按弧长比例（或按索引比例兜底）
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);         // z 线性插值
      }

      points.front()(2) = start_z;   // 强制首点 z = 起始 z
      points.back()(2) = target_z;   // 强制末点 z = 目标 z
    }
  } // namespace

  // SECTION interfaces for setup and query    // 接口区段：初始化与查询

  SCANPlannerManager::SCANPlannerManager() {}    // 构造函数（空实现）

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }   // 析构函数（打印信息）

  // 初始化规划模块：读取算法参数、创建栅格地图与 B 样条优化器、绑定可视化
  void SCANPlannerManager::initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis)
  {
    node_ = node;                    // 保存节点指针
    /* read algorithm parameters */  // 读取算法参数
    const auto get_double = [node](const std::string &name, double default_value) {   // 读取 double 参数（缺失则声明默认值）
      if (!node->has_parameter(name)) node->declare_parameter<double>(name, default_value);
      return node->get_parameter(name).as_double();
    };
    pp_.max_vel_ = get_double("manager.max_vel", -1.0);      // 最大速度 [m/s]
    pp_.max_acc_ = get_double("manager.max_acc", -1.0);      // 最大加速度 [m/s²]
    pp_.max_jerk_ = get_double("manager.max_jerk", -1.0);    // 最大加加速度 [m/s³]
    pp_.vel_tolerance_ = get_double("optimization.vel_tolerance", 1.0);   // 速度容差比例
    pp_.acc_tolerance_ = get_double("optimization.acc_tolerance", 1.0);   // 加速度容差比例
    pp_.feasibility_tolerance_ = get_double("manager.feasibility_tolerance", 0.0);   // 可行性容差（允许超限比例）
    pp_.ctrl_pt_dist = get_double("manager.control_points_distance", -1.0);   // 控制点间距 [m]
    pp_.planning_horizon_ = get_double("manager.planning_horizon", 5.0);      // 规划视界 [m]

    local_data_.traj_id_ = 0;        // 初始化局部轨迹 ID
    grid_map_.reset(new GridMap);    // 创建栅格地图
    grid_map_->initMap(node_);       // 初始化地图（读取地图参数/建立网格）
    collision_map_ = std::make_shared<m20_trajectory::ScanMapAdapter>(grid_map_);

    bspline_optimizer_rebound_.reset(new m20_trajectory::ScanOptimizerAdapter);
    bspline_optimizer_rebound_->configure(node_, grid_map_);
    bspline_optimizer_rebound_->backend().a_star_.reset(new AStar);
    bspline_optimizer_rebound_->backend().a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;            // 绑定可视化对象
  }

  // !SECTION    // 接口区段结束

  // SECTION rebond replanning    // 反弹重规划区段

  // 反弹重规划主接口：前端初始化（多项式/A*） + 后端优化（B 样条）生成局部轨迹
  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;            // 重规划次数计数器（静态）
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;   // 打印分隔线
    cout.precision(3);               // 打印精度
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;                    // 打印起止状态

    if ((start_pt - local_target_pt).norm() < 0.2)   // 起点距目标过近（<0.2m）
    {
      cout << "Close to goal" << endl;   // 打印提示
      continuous_failures_count_++;      // 失败计数 +1
      return false;                      // 无需规划
    }

    auto t_start = std::chrono::steady_clock::now();   // 记录阶段起始时间
    double t_init = 0.0, t_opt = 0.0, t_refine = 0.0; // 初始化各阶段耗时

    /*** STEP 1: INIT ***/    // 第一步：前端初始化
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    // 采样时间步长：距离较远取 ctrl_pt_dist/max_vel*1.2，很近取 *5（乘 5 更宽松；ctrl_pt_dist/max_vel 太紧必然超限）
    vector<Eigen::Vector3d> point_set, start_end_derivatives;   // 初始路径点集 与 起止导数集合
    static bool flag_first_call = true, flag_force_polynomial = false;   // 首次调用标志 / 强制多项式初始化标志
    bool flag_regenerate = false;        // 需要重新生成标志
    do
    {
      point_set.clear();                 // 清空路径点
      start_end_derivatives.clear();     // 清空导数
      flag_regenerate = false;           // 重置重生成标志

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      // 首次调用 / 要求多项式初始化 / 强制多项式：初始路径由 min-snap 轨迹按阶采样生成
      {
        flag_first_call = false;         // 清除首次调用标志
        flag_force_polynomial = false;   // 清除强制多项式标志

        m20_trajectory::PolynomialTraj gl_traj;          // 全局多项式初始轨迹

        double dist = (start_pt - local_target_pt).norm();   // 起点到目标距离
        double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;
        // 梯形速度剖面总时间：加速/减速段 + 匀速段

        if (!flag_randomPolyTraj)        // 非随机初始路径：直接生成单段多项式轨迹
        {
          gl_traj = m20_trajectory::PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else                             // 随机初始路径：插入随机中间点后做 min-snap
        {
          Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();   // 水平扰动方向
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();               // 垂直扰动方向
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +    // 随机中间点 = 中点 + 随机扰动（幅度随失败次数衰减）
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);     // 3 个途经点：起点/随机点/终点
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);          // 两段时间（各为总时间一半）
          t(0) = t(1) = time / 2;
          gl_traj = m20_trajectory::PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);   // min-snap 生成
        }

        double t;                        // 采样时刻
        bool flag_too_far;               // 采样点间距过大标志
        ts *= 1.5; // ts will be divided by 1.5 in the next  先放大 1.5 倍（下一步再除回）
        do
        {
          ts /= 1.5;                     // 逐步收紧采样步长
          point_set.clear();             // 清空路径点
          flag_too_far = false;          // 重置间距标志
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);   // 上一点（从起点开始）
          for (t = 0; t < time; t += ts) // 按步长采样多项式轨迹
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);      // 当前采样点
            if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)   // 相邻点距离超过 1.5 倍控制点间距
            {
              flag_too_far = true;       // 标记间距过大
              break;                     // 提前终止（收紧步长重试）
            }
            last_pt = pt;                // 更新上一点
            point_set.push_back(pt);     // 收集路径点
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        // 直到间距达标且点数不少于 7（保证初始路径有足够多的点）
        t -= ts;                         // 末点时刻（回退一个步长）
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));   // 起点速度
        start_end_derivatives.push_back(local_target_vel);         // 终点速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));   // 起点加速度
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));   // 终点加速度
      }
      else // Initial path generated from previous trajectory.
      // 否则：初始路径由前一条局部轨迹采样生成
      {

        double t;                        // 采样时刻
        double t_cur = (node_->now() - local_data_.start_time_).seconds();   // 当前相对局部轨迹起始的时间

        vector<double> pseudo_arc_length;        // 伪弧长（各采样点的累计长度）
        vector<Eigen::Vector3d> segment_point;   // 采样点集
        pseudo_arc_length.push_back(0.0);        // 起点弧长 = 0
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)   // 沿当前局部轨迹采样
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));   // 采样位置点
          if (t > t_cur)                 // 非首个点：累加弧长
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;                         // 末点时刻

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;   // 到目标的衔接多项式时长（按 2 倍速度反推）
        if (poly_time > ts)              // 衔接段长于一个步长：补充生成多项式段
        {
          m20_trajectory::PolynomialTraj gl_traj = m20_trajectory::PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),   // 单段多项式衔接轨迹
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)   // 采样衔接多项式段并追加到路径
          {
            if (!pseudo_arc_length.empty())      // 弧长容器非空
            {
              segment_point.push_back(gl_traj.evaluate(t));   // 追加采样点
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());   // 累加弧长
            }
            else
            {
              RCLCPP_ERROR(node_->get_logger(), "pseudo_arc_length is empty; aborting replan");   // 弧长为空：中止重规划
              continuous_failures_count_++;      // 失败计数 +1
              return false;
            }
          }
        }

        double sample_length = 0;        // 当前采样弧长
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next  控制点间距先放大 1.5 倍（下一步除回）
        size_t id = 0;                   // 弧长段索引
        do
        {
          cps_dist /= 1.5;               // 逐步收紧采样间距
          point_set.clear();             // 清空路径点
          sample_length = 0;             // 重置采样弧长
          id = 0;                        // 重置段索引
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())   // 按弧长遍历各段
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])   // 落在段 [id, id+1] 内
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);   // 段内线性插值
              sample_length += cps_dist;          // 推进采样弧长
            }
            else
              id++;                      // 进入下一段
          }
          point_set.push_back(local_target_pt);   // 追加目标点
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help
        // 直到点数不少于 7（起点与终点很接近时此循环保证点数）

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));   // 起点速度（当前轨迹速度）
        start_end_derivatives.push_back(local_target_vel);                                     // 终点速度
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));   // 起点加速度
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());                              // 终点加速度（0）

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!  初始路径异常过长
        {
          flag_force_polynomial = true;  // 强制下次用多项式初始化
          flag_regenerate = true;        // 标记重新生成
        }
      }
    } while (flag_regenerate);           // 若需重新生成则循环

    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));   // 为路径点线性插值 z 参考

    Eigen::MatrixXd ctrl_pts;                                            // 控制点矩阵（输出）
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);   // 由路径点参数化 B 样条（反解控制点）

    vector<vector<Eigen::Vector3d>> a_star_paths;                        // A* 路径（多层）
    a_star_paths = bspline_optimizer_rebound_->backend().initControlPoints(ctrl_pts, true);

    t_init = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();   // 记录初始化耗时

    static int vis_id = 0;               // 可视化 ID 计数器
    visualization_->displayInitPathList(point_set, 0.2, 0);   // 显示初始路径
    visualization_->displayAStarList(a_star_paths, vis_id);   // 显示 A* 调整路径

    t_start = std::chrono::steady_clock::now();   // 重新计时（优化阶段）

    /*** STEP 2: OPTIMIZE ***/   // 第二步：后端优化
    bool flag_step_1_success = bspline_optimizer_rebound_->optimize(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;   // 打印优化结果
    if (!flag_step_1_success)          // 优化失败
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;    // 失败计数 +1
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();   // 记录优化耗时
    t_start = std::chrono::steady_clock::now();   // 重新计时（精化阶段）

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/   // 第三步：必要时时间重分配（精化）
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);   // 由优化后的控制点构造三次 B 样条
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);   // 设置物理限制

    double ratio;                       // 可行性比值（输出）
    bool flag_step_2_success = true;    // 精化成功标志（默认成功）
    if (!pos.checkFeasibility(ratio, false))   // 速度/加速度可行性检查失败
    {
      cout << "Need to reallocate time." << endl;   // 需要时间重分配

      Eigen::MatrixXd optimal_control_points;      // 精化后的最优控制点
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);   // 时间重分配 + 再次优化
      if (flag_step_2_success)                     // 精化成功
        pos = UniformBspline(optimal_control_points, 3, ts);   // 用精化后的控制点重建样条
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos))   // 精化失败或动态可行性检查失败
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");   // 打印警告（蓝色）
      continuous_failures_count_++;    // 失败计数 +1
      return false;
    }

    t_refine = std::chrono::duration<double>(std::chrono::steady_clock::now() - t_start).count();   // 记录精化耗时

    // save planned results
    updateTrajInfo(pos, node_->now());   // 保存规划结果到局部轨迹数据

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine)   // 打印总耗时/优化耗时/精化耗时
         << "\033[0m,optimize:" << (t_init + t_opt) << ",refine:" << t_refine << endl;

    // success. YoY
    continuous_failures_count_ = 0;     // 重置失败计数
    return true;
  }

  // 紧急停车：生成停在 stop_pos 的轨迹（6 个相同控制点）
  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);   // 6 个控制点
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;     // 全部取停车位置（原地不动）
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());   // 更新为原地停止轨迹

    return true;
  }

  // 路点序列全局轨迹规划：由路点生成途经的 min-snap 全局参考轨迹
  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory
    // 生成全局参考轨迹

    if (waypoints.empty())      // 路点为空
      return false;

    vector<Eigen::Vector3d> points;      // 途经点序列（起点 + 路点）
    points.push_back(start_pos);         // 加入起点

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]); // 依次加入路点
    }

    double total_len = 0;                // 总长度
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();   // 累加相邻点距离
    }

    // insert intermediate points if too far
    // 若相邻点过远则插入中间点（加密）
    vector<Eigen::Vector3d> inter_points;    // 插值后的点列
    double dist_thresh = max(total_len / 8, 4.0);   // 加密距离阈值（总长/8 与 4m 取大）

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));   // 保留原有点
      double dist = (points.at(i + 1) - points.at(i)).norm();   // 相邻点距离

      if (dist > dist_thresh)                 // 距离超阈值：插入等分点
      {
        int id_num = floor(dist / dist_thresh) + 1;   // 分段数

        for (int j = 1; j < id_num; ++j)      // 插入 j 个中间点
        {
          Eigen::Vector3d inter_pt =          // 按比例线性插值
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());    // 追加末点

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    // 写入位置矩阵
    int pt_num = inter_points.size();         // 点数
    Eigen::MatrixXd pos(3, pt_num);           // 位置矩阵（3 × N）
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];           // 逐列填充

    Eigen::Vector3d zero(0, 0, 0);            // 零向量
    Eigen::VectorXd time(pt_num - 1);         // 各段时间
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);   // 段时长 = 距离 / 最大速度
    }

    time(0) *= 2.0;                           // 首段时间加倍（加减速段）
    time(time.rows() - 1) *= 2.0;             // 末段时间加倍（加减速段）

    m20_trajectory::PolynomialTraj gl_traj;                   // 全局多项式轨迹
    if (pos.cols() >= 3)                      // 3 个以上点：min-snap 轨迹
      gl_traj = m20_trajectory::PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)                 // 2 个点：单段轨迹
      gl_traj = m20_trajectory::PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;                           // 点数不足

    auto time_now = node_->now();             // 当前时刻
    global_data_.setGlobalTraj(gl_traj, time_now);   // 保存为全局轨迹

    return true;
  }

  // 单目标全局轨迹规划：起点到终点的 min-snap 全局参考轨迹
  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory
    // 生成全局参考轨迹

    vector<Eigen::Vector3d> points;      // 途经点（仅起终点）
    points.push_back(start_pos);         // 起点
    points.push_back(end_pos);           // 终点

    // insert intermediate points if too far
    // 若相距过远则插入中间点（加密）
    vector<Eigen::Vector3d> inter_points;    // 插值后的点列
    const double dist_thresh = 4.0;          // 加密距离阈值 [m]

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));  // 保留原有点
      double dist = (points.at(i + 1) - points.at(i)).norm();   // 相邻点距离

      if (dist > dist_thresh)                // 距离超阈值：插入等分点
      {
        int id_num = floor(dist / dist_thresh) + 1;   // 分段数

        for (int j = 1; j < id_num; ++j)     // 插入 j 个中间点
        {
          Eigen::Vector3d inter_pt =         // 按比例线性插值
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());   // 追加末点

    // write position matrix
    // 写入位置矩阵
    int pt_num = inter_points.size();        // 点数
    Eigen::MatrixXd pos(3, pt_num);          // 位置矩阵（3 × N）
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];          // 逐列填充

    Eigen::Vector3d zero(0, 0, 0);           // 零向量
    Eigen::VectorXd time(pt_num - 1);        // 各段时间
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);   // 段时长 = 距离 / 最大速度
    }

    time(0) *= 2.0;                          // 首段时间加倍
    time(time.rows() - 1) *= 2.0;            // 末段时间加倍

    m20_trajectory::PolynomialTraj gl_traj;                  // 全局多项式轨迹
    if (pos.cols() >= 3)                     // 3 个以上点：min-snap 轨迹
      gl_traj = m20_trajectory::PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)                // 2 个点：单段轨迹
      gl_traj = m20_trajectory::PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;                          // 点数不足

    auto time_now = node_->now();            // 当前时刻
    global_data_.setGlobalTraj(gl_traj, time_now);   // 保存为全局轨迹

    return true;
  }

  // 轨迹精化算法：时间重分配（reparamBspline）后重新采样并再次优化
  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;                            // 时间增量（输出）

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()   控制点矩阵（输出）

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);   // 时间重分配（拉长时间降低速度/加速度）

    traj = UniformBspline(ctrl_pts, 3, ts);  // 用重分配后的控制点重建样条

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);   // 采样步长（按控制点数均分时长）
    bspline_optimizer_rebound_->backend().ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->backend().ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->refine(ctrl_pts, ts, bspline_optimizer_rebound_->backend().ref_pts_);
    optimal_control_points = ctrl_pts;

    return success;                          // 返回优化结果
  }

  // 更新局部轨迹信息：保存位置/速度/加速度样条与执行信息
  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
        local_data_.start_time_ = time_now;      // 记录起始时刻
    local_data_.position_traj_ = position_traj;   // 位置样条
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();   // 速度样条（一阶导）
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();   // 加速度样条（二阶导）
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);   // 起始位置
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();            // 轨迹时长
    local_data_.traj_id_ += 1;               // 轨迹 ID +1
  }

  // 动态可行性检查：对速度/加速度按采样点逐点校验是否超限（含容差）
  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();     // 速度样条
    UniformBspline acc_traj = vel_traj.getDerivative();          // 加速度样条
    const double duration = position_traj.getTimeSum();          // 轨迹时长
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));   // 采样步长（10~50ms，总样本约 50 个）
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;  // 速度上限（含容差）
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;  // 加速度上限（含容差）

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)    // 逐点采样检查
    {
      const double tc = std::min(t, duration);                   // 截断到轨迹末
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);        // 采样速度
      if (vel.norm() > vel_limit)                                // 速度超限
      {
        RCLCPP_WARN(node_->get_logger(),                         // 打印警告并返回失败
                    "Dynamic feasibility failed: velocity at t=%.3f is %.3f > %.3f",
                    tc, vel.norm(), vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);        // 采样加速度
      if (acc.norm() > acc_limit)                                // 加速度超限
      {
        RCLCPP_WARN(node_->get_logger(),                         // 打印警告并返回失败
                    "Dynamic feasibility failed: acceleration at t=%.3f is %.3f > %.3f",
                    tc, acc.norm(), acc_limit);
        return false;
      }
    }

    return true;                                                 // 全部通过
  }

  // 时间重分配：按比例拉长 B 样条时间（降低速度/加速度），并重新拟合控制点
  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();                   // 原始时长
    int seg_num = bspline.getControlPoint().cols() - 3;          // 分段数（控制点数 - 3）
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);                                 // 按比例拉长时间
    double duration = bspline.getTimeSum();                      // 新时长
    dt = duration / double(seg_num);                             // 新的采样步长
    time_inc = duration - time_origin;                           // 时间增量

    vector<Eigen::Vector3d> point_set;                           // 采样点集
    for (double time = 0.0; time <= duration + 1e-4; time += dt) // 按新步长采样
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));        // 采样位置点
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);   // 反解新控制点
  }

} // namespace scan_planner
