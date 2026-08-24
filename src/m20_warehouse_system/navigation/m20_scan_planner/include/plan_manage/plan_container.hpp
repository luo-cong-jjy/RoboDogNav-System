/**
 * @file plan_container.hpp
 * @brief 轨迹数据容器与规划参数结构（GlobalTrajData / LocalTrajData / PlanParameters）
 *
 * 职责：
 *  - GlobalTrajData：管理"全局参考轨迹 + 局部 B 样条轨迹"的拼接与按时间查询
 *    （位置/速度/加速度），并维护局部轨迹插入导致的全局时间轴偏移（time_increase_）；
 *  - LocalTrajData：保存当前正在执行的局部轨迹（位置/速度/加速度 B 样条）
 *    及其起始时间、轨迹 ID 等执行信息；
 *  - PlanParameters：规划算法参数（速度/加速度/加加速度上限、控制点间距、
 *    可行性容差、规划视界、各阶段耗时统计）。
 *
 * 本头文件为扫描规划器（scan_planner）的核心数据结构，被 planner_manager.h 等引用。
 */
#ifndef _PLAN_CONTAINER_H_    // 头文件保护宏：防止重复包含
#define _PLAN_CONTAINER_H_    // 定义保护宏

#include <Eigen/Eigen>            // Eigen 线性代数库：向量/矩阵/四元数等
#include <vector>                 // 标准容器：std::vector（动态数组）
#include <rclcpp/rclcpp.hpp>      // ROS2 C++ 客户端库：rclcpp::Time 时间类型等

#include <bspline_opt/uniform_bspline.h>
#include "m20_trajectory/polynomial_traj.h"

using std::vector;    // 引入 std::vector 简写

namespace scan_planner    // 扫描规划器命名空间
{

  // 全局轨迹数据：全局多项式参考轨迹 + 局部 B 样条轨迹的时间轴管理容器
  class GlobalTrajData
  {
  private:
  public:
    m20_trajectory::PolynomialTraj global_traj_;
    vector<UniformBspline> local_traj_;       // 局部轨迹及其导数：[0]=位置 [1]=速度 [2]=加速度

    double global_duration_;                  // 全局轨迹总时长 [s]
    rclcpp::Time global_start_time_;          // 全局轨迹起始时刻（ROS 时间）
    double local_start_time_, local_end_time_;  // 局部轨迹的起止时间 [s]
    double time_increase_;                    // 累计时间增量（局部轨迹插入导致全局时长的增长量）
    double last_time_inc_;                    // 最近一次插入局部轨迹带来的时间增量
    double last_progress_time_;               // 全局轨迹进度时间（用于计算局部目标点）

    GlobalTrajData(/* args */) {}             // 默认构造函数（无初始化逻辑）

    ~GlobalTrajData() {}                      // 默认析构函数

    // 判断局部轨迹是否已到达全局轨迹终点（时间差小于 0.1s 视为到达）
    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    // 设置全局轨迹：保存轨迹并重置局部轨迹相关的所有时间状态
    void setGlobalTraj(const m20_trajectory::PolynomialTraj &traj, const rclcpp::Time &time)
    {
      global_traj_ = traj;              // 保存全局轨迹
      global_traj_.init();              // 初始化轨迹（更新内部时间参数）
      global_duration_ = global_traj_.getTimeSum();   // 获取全局轨迹总时长
      global_start_time_ = time;        // 记录全局轨迹起始时刻

      local_traj_.clear();              // 清空局部轨迹
      local_start_time_ = -1;           // 重置局部起止时间（表示暂无局部轨迹）
      local_end_time_ = -1;
      time_increase_ = 0.0;             // 重置时间增量
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;        // 重置进度时间
    }

    // 设置局部轨迹：保存位置/速度/加速度样条，并把时间增量并入全局时长
    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3);            // 预分配 3 段（位置/速度/加速度）
      local_traj_[0] = traj;            // 位置样条
      local_traj_[1] = local_traj_[0].getDerivative();   // 速度样条（位置样条的一阶导）
      local_traj_[2] = local_traj_[1].getDerivative();   // 加速度样条（速度样条的一阶导）

      local_start_time_ = local_ts;     // 局部轨迹起始时间
      local_end_time_ = local_te;       // 局部轨迹结束时间
      global_duration_ += time_inc;     // 全局时长增加（局部轨迹带来的时间偏移）
      time_increase_ += time_inc;       // 累计时间增量
      last_time_inc_ = time_inc;        // 记录本次增量
    }

    // 按全局时间 t 查询位置：局部轨迹区间内用局部样条求值，其余区间用全局轨迹求值
    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)    // 在局部轨迹开始之前：用全局轨迹
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_);
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)   // 在局部轨迹结束之后：用全局轨迹（时间轴扣除增量）
      {
        return global_traj_.evaluate(t - time_increase_);
      }
      else    // 处于局部轨迹区间内：用局部位置样条求值
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);    // 获取局部样条的时间跨度
        return local_traj_[0].evaluateDeBoorT(tm + t - local_start_time_);   // 把全局时间映射到局部时间后求值
      }
    }

    // 按全局时间 t 查询速度：逻辑同 getPosition，但使用速度样条/全局轨迹速度
    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)    // 局部轨迹开始之前
      {
        return global_traj_.evaluateVel(t);        // 全局轨迹速度
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)   // 局部轨迹结束之后
      {
        return global_traj_.evaluateVel(t - time_increase_);   // 全局轨迹速度（扣除时间增量）
      }
      else    // 局部轨迹区间内
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);    // 获取局部样条时间跨度
        return local_traj_[1].evaluateDeBoorT(tm + t - local_start_time_);   // 局部速度样条求值
      }
    }

    // 按全局时间 t 查询加速度：逻辑同 getPosition/getVelocity，使用加速度样条
    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)    // 局部轨迹开始之前
      {
        return global_traj_.evaluateAcc(t);        // 全局轨迹加速度
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)   // 局部轨迹结束之后
      {
        return global_traj_.evaluateAcc(t - time_increase_);   // 全局轨迹加速度（扣除时间增量）
      }
      else    // 局部轨迹区间内
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);    // 获取局部样条时间跨度
        return local_traj_[2].evaluateDeBoorT(tm + t - local_start_time_);   // 局部加速度样条求值
      }
    }

    // 获取球半径范围内的局部轨迹 B 样条参数化数据
    // start_t: 轨迹起始时间
    // dist_pt: 离散点间距
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // length of the truncated segment  截取段的弧长
      double seg_time = 0.0;   // duration of the truncated segment  截取段的时长
      double radius = 0.0;     // distance to the first point of the segment  当前点到段首点的距离

      double delta = 0.2;      // 采样步长 [s]
      Eigen::Vector3d first_pt = getPosition(start_t); // first point of the segment  段首点
      Eigen::Vector3d prev_pt = first_pt;              // previous point  上一个采样点
      Eigen::Vector3d cur_pt;                          // current point  当前采样点

      // go forward until the traj exceed radius or global time
      // 向前推进采样，直到半径超过设定值或时间超出全局轨迹

      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delta;                        // 推进一个采样步长
        seg_time = min(seg_time, global_duration_ - start_t);   // 限制不超过全局轨迹剩余时长

        cur_pt = getPosition(start_t + seg_time); // 采样当前时刻的位置
        seg_length += (cur_pt - prev_pt).norm();  // 累加段内弧长
        prev_pt = cur_pt;                         // 更新上一个采样点
        radius = (cur_pt - first_pt).norm();      // 更新到段首点的距离
      }

      // get parameterization dt by desired density of points
      // 根据期望的点密度计算离散时间步长
      int seg_num = floor(seg_length / dist_pt);  // 段内离散点数

      // get outputs
      // 输出结果

      seg_duration = seg_time; // duration of the truncated segment  截取段时长
      dt = seg_time / seg_num; // time difference between two points  相邻两点的时间间隔

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)   // 按 dt 离散采样并收集位置点
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));           // 段首速度
      start_end_derivative.push_back(getVelocity(start_t + seg_time)); // 段末速度
      start_end_derivative.push_back(getAcceleration(start_t));        // 段首加速度
      start_end_derivative.push_back(getAcceleration(start_t + seg_time));   // 段末加速度
    }

    // 获取固定时长局部轨迹的 B 样条参数化数据
    // start_t: start time of the trajectory  轨迹起始时间
    // duration: time length of the segment  段时长
    // seg_num: discretized the segment into *seg_num* parts  将段离散为 seg_num 份
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num;          // 由段时长与份数计算离散时间步长
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)   // 按 dt 离散采样并收集位置点
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt);
      }

      start_end_derivative.push_back(getVelocity(start_t));            // 段首速度
      start_end_derivative.push_back(getVelocity(start_t + duration));  // 段末速度
      start_end_derivative.push_back(getAcceleration(start_t));         // 段首加速度
      start_end_derivative.push_back(getAcceleration(start_t + duration));   // 段末加速度
    }
  };

  // 规划算法参数结构体：集中存放物理限制、容差与耗时统计
  struct PlanParameters
  {
    /* planning algorithm parameters */   // 规划算法参数
    double max_vel_, max_acc_, max_jerk_; // physical limits  物理限制：最大速度/加速度/加加速度
    double vel_tolerance_, acc_tolerance_;   // 速度/加速度可行性容差
    double ctrl_pt_dist;                  // distance between adjacient B-spline control points  相邻 B 样条控制点间距
    double feasibility_tolerance_;        // permitted ratio of vel/acc exceeding limits  允许超出限制的速度/加速度比例
    double planning_horizon_;             // 局部规划视界 [m]

    /* processing time */                 // 处理耗时统计
    double time_search_ = 0.0;            // 前端搜索耗时
    double time_optimize_ = 0.0;          // 后端优化耗时
    double time_adjust_ = 0.0;            // 时间重分配（refine）耗时
  };

  // 局部轨迹数据：当前正在执行/发布的局部轨迹及其执行信息
  struct LocalTrajData
  {
    /* info of generated traj */          // 生成轨迹的信息

    int traj_id_;                         // 轨迹 ID（单调递增）
    double duration_;                     // 轨迹时长 [s]
    double global_time_offset; // This is because when the local traj finished and is going to switch back to the global traj, the global traj time is no longer matches the world time.
                                // 全局时间偏移：局部轨迹结束并切回全局轨迹时，全局轨迹时间与真实时间不再对齐，用该偏移修正
    rclcpp::Time start_time_;             // 轨迹起始时刻（ROS 时间）
    Eigen::Vector3d start_pos_;           // 轨迹起始位置
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_;   // 位置/速度/加速度 B 样条
  };

} // namespace scan_planner

#endif    // 头文件保护宏结束
