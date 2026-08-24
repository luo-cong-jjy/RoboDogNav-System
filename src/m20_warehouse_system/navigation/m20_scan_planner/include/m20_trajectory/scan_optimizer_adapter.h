#pragma once

#include "m20_trajectory/trajectory_optimizer_boundary.h"
#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <rclcpp/rclcpp.hpp>

namespace m20_trajectory {

// Behaviour-preserving bridge for the current SCAN backend.  Keeping this
// class at the M20 boundary lets the backend be replaced without changing the
// planner state machine or its trajectory contracts.
class ScanOptimizerAdapter final : public TrajectoryOptimizerBoundary {
 public:
  using Ptr = std::shared_ptr<ScanOptimizerAdapter>;
  void configure(rclcpp::Node *node, const GridMap::Ptr &map) {
    optimizer_.setParam(node);
    optimizer_.setEnvironment(map);
  }
  Path initialize(const Eigen::MatrixXd &seed) override {
    Eigen::MatrixXd points = seed;
    const auto paths = optimizer_.initControlPoints(points, true);
    return paths.empty() ? Path{} : paths.back();
  }
  bool optimize(Eigen::MatrixXd &points, double &interval) override {
    return optimizer_.BsplineOptimizeTrajRebound(points, interval);
  }
  bool refine(Eigen::MatrixXd &points, double &interval,
              const Path &reference_points) override {
    optimizer_.ref_pts_ = reference_points;
    Eigen::MatrixXd refined;
    if (!optimizer_.BsplineOptimizeTrajRefine(points, interval, refined)) return false;
    points = refined;
    return true;
  }
  scan_planner::BsplineOptimizer &backend() { return optimizer_; }

 private:
  scan_planner::BsplineOptimizer optimizer_;
};

}  // namespace m20_trajectory
