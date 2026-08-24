#pragma once

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace m20_trajectory {

// Stable M20 planning contract.  The current SCAN adapter can implement this
// boundary while the optimizer and map backends are replaced independently.
class TrajectoryOptimizerBoundary {
 public:
  using Path = std::vector<Eigen::Vector3d>;
  virtual ~TrajectoryOptimizerBoundary() = default;
  virtual bool optimize(Eigen::MatrixXd &control_points,
                        double &knot_interval) = 0;
  virtual bool refine(Eigen::MatrixXd &control_points,
                      double &knot_interval,
                      const Path &reference_points) = 0;
  virtual Path initialize(const Eigen::MatrixXd &seed_control_points) = 0;
};

}  // namespace m20_trajectory
