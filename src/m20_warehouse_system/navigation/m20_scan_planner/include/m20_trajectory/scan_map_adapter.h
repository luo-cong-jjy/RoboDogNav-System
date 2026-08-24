#pragma once

#include "m20_trajectory/map_collision_boundary.h"
#include <plan_env/grid_map.h>

namespace m20_trajectory {

class ScanMapAdapter final : public MapCollisionBoundary {
 public:
  explicit ScanMapAdapter(const GridMap::Ptr &map) : map_(map) {}
  void reset() override { if (map_) map_->resetBuffer(); }
  bool occupied(const Eigen::Vector3d &position,
                double heading = 0.0) const override {
    return map_ && map_->getInflateOccupancy(position, heading) == 1;
  }
  bool inside(const Eigen::Vector3d &position) const override {
    return map_ && map_->isInMap(position);
  }
  GridMap::Ptr backend() const { return map_; }

 private:
  GridMap::Ptr map_;
};

}  // namespace m20_trajectory
