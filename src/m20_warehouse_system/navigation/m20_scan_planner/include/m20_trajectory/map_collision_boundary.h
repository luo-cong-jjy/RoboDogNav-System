#pragma once
#include <Eigen/Core>
#include <memory>

namespace m20_trajectory {

class MapCollisionBoundary {
 public:
  using Ptr = std::shared_ptr<MapCollisionBoundary>;
  virtual ~MapCollisionBoundary() = default;
  virtual void reset() = 0;
  virtual bool occupied(const Eigen::Vector3d &position,
                        double heading = 0.0) const = 0;
  virtual bool inside(const Eigen::Vector3d &position) const = 0;
};

}  // namespace m20_trajectory
