/**
 * @file planner_manager.h
 * @brief 扫描规划管理器（SCANPlannerManager）头文件
 *
 * 职责：
 *  - 统一封装规划系统的核心算法：前端初始化（多项式/A* 路径）与后端优化
 *    （B 样条优化器），提供"反弹重规划"（reboundReplan）主接口；
 *  - 提供全局轨迹规划（单目标 planGlobalTraj / 路点 planGlobalTrajWaypoints）
 *    与紧急停车（EmergencyStop）接口；
 *  - 管理规划所需的数据容器（PlanParameters / LocalTrajData / GlobalTrajData）
 *    与栅格地图（GridMap）、可视化（PlanningVisualization）；
 *  - 内部完成轨迹可行性检查、时间重分配（reparamBspline / refineTrajAlgo）
 *    与局部轨迹信息更新（updateTrajInfo）。
 *
 * 为 planner_manager.cpp 的实现声明接口。
 */
#ifndef _PLANNER_MANAGER_H_    // 头文件保护宏：防止重复包含
#define _PLANNER_MANAGER_H_    // 定义保护宏

#include <stdlib.h>             // 标准库：rand（随机初始路径生成）

#include <bspline_opt/bspline_optimizer.h>      // B 样条优化器：BsplineOptimizer（后端优化/前端 A*）
#include <bspline_opt/uniform_bspline.h>        // 均匀 B 样条：UniformBspline（轨迹表示）
#include <plan_env/grid_map.h>                  // 栅格地图：GridMap（碰撞环境）
#include <plan_manage/plan_container.hpp>       // 轨迹数据容器：GlobalTrajData/LocalTrajData/PlanParameters
#include <rclcpp/rclcpp.hpp>                    // ROS2 C++ 客户端库：Node 等
#include <traj_utils/planning_visualization.h>  // 规划可视化：PlanningVisualization

namespace scan_planner    // 扫描规划器命名空间
{

  // Fast Planner Manager
  // Key algorithms of mapping and planning are called
  // 规划管理器：负责调度映射与规划的关键算法（前端初始化、后端优化、全局轨迹规划）

  class SCANPlannerManager
  {
    // SECTION stable    // 稳定接口区段
  public:
    SCANPlannerManager();       // 构造函数
    ~SCANPlannerManager();      // 析构函数

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 使类内 Eigen 成员内存 16 字节对齐

    /* main planning interface */   // 主规划接口
    // 反弹重规划：以前端（多项式/A*）初始路径 + 后端（B 样条优化）生成局部轨迹
    bool reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d start_acc,
                       Eigen::Vector3d end_pt, Eigen::Vector3d end_vel, bool flag_polyInit, bool flag_randomPolyTraj);
    bool EmergencyStop(Eigen::Vector3d stop_pos);   // 紧急停车：生成停在当前点的轨迹
    // 全局轨迹规划（单目标点）：min-snap 多项式轨迹
    bool planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                        const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);
    // 全局轨迹规划（路点序列）：途经多个路点的 min-snap 多项式轨迹
    bool planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                 const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initPlanModules(rclcpp::Node *node, PlanningVisualization::Ptr vis = nullptr);   // 初始化规划模块（读参数/建地图/建优化器）

    PlanParameters pp_;             // 规划算法参数
    LocalTrajData local_data_;      // 当前局部轨迹数据
    GlobalTrajData global_data_;    // 全局轨迹数据
    GridMap::Ptr grid_map_;         // 栅格地图指针

  private:
    rclcpp::Node *node_{nullptr};   // 所属 ROS2 节点指针
    /* main planning algorithms & modules */   // 核心规划算法与模块
    PlanningVisualization::Ptr visualization_;      // 可视化对象
    BsplineOptimizer::Ptr bspline_optimizer_rebound_;   // 反弹重规划使用的 B 样条优化器

    int continuous_failures_count_{0};      // 连续规划失败计数（用于随机初始路径扰动强度）

    void updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now);   // 更新局部轨迹信息
    bool checkDynamicFeasibility(UniformBspline position_traj);    // 检查轨迹动态可行性（速度/加速度采样校验）

    // 时间重参数化：按比例拉长时间以降低速度/加速度，并重新拟合控制点
    void reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio, Eigen::MatrixXd &ctrl_pts, double &dt,
                        double &time_inc);

    // 轨迹精化：时间重分配后再次优化，输出最优控制点
    bool refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points);

    // !SECTION stable    // 稳定接口区段结束

    // SECTION developing   // 开发中区段

  public:
    typedef unique_ptr<SCANPlannerManager> Ptr;   // 智能指针类型别名（管理器指针）

    // !SECTION     // 开发中区段结束
  };
} // namespace scan_planner

#endif    // 头文件保护宏结束
