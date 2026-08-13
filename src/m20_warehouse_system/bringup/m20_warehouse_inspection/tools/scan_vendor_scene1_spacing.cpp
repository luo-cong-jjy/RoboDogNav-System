// Copyright 2026 Virdyn Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Reproduce the deterministic box sequence used by the vendored SCAN-Planner
// Mockamap scene-1 configuration.  This is intentionally a small standalone
// C++ program because Mockamap uses libstdc++ std::default_random_engine; using
// NumPy or Python's random generator would not reproduce the same scene.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

namespace
{
struct Box
{
  double x;
  double y;
  double width;
};

double separation(const Box & left, const Box & right)
{
  const double half_sum = 0.5 * (left.width + right.width);
  const double dx = std::max(std::abs(left.x - right.x) - half_sum, 0.0);
  const double dy = std::max(std::abs(left.y - right.y) - half_sum, 0.0);
  return std::hypot(dx, dy);
}
}  // namespace

int main()
{
  constexpr int obstacle_count = 500;
  std::default_random_engine engine(127);
  std::uniform_real_distribution<double> random_x(-20.0, 20.0);
  std::uniform_real_distribution<double> random_y(-20.0, 20.0);
  std::uniform_real_distribution<double> random_width(0.2, 0.8);
  std::uniform_real_distribution<double> random_height(2.0, 2.0);

  std::vector<Box> boxes;
  boxes.reserve(obstacle_count);
  for (int index = 0; index < obstacle_count; ++index) {
    const double x = random_x(engine);
    const double y = random_y(engine);
    const double width = random_width(engine);
    (void)random_height(engine);
    boxes.push_back({x, y, width});
  }

  std::vector<int> parent(obstacle_count);
  std::iota(parent.begin(), parent.end(), 0);
  auto root = [&](int value) {
      int current = value;
      while (parent[current] != current) {
        current = parent[current];
      }
      while (parent[value] != value) {
        const int next = parent[value];
        parent[value] = current;
        value = next;
      }
      return current;
    };
  auto join = [&](int left, int right) {
      left = root(left);
      right = root(right);
      if (left != right) {
        parent[right] = left;
      }
    };

  int overlapping_pairs = 0;
  int positive_gap_below_010 = 0;
  int positive_gap_below_050 = 0;
  int positive_gap_below_090 = 0;
  double minimum_positive_pair_gap = INFINITY;
  double primitive_area = 0.0;
  double mean_width = 0.0;
  for (const auto & box : boxes) {
    primitive_area += box.width * box.width;
    mean_width += box.width;
  }
  mean_width /= boxes.size();

  for (int left = 0; left < obstacle_count; ++left) {
    for (int right = left + 1; right < obstacle_count; ++right) {
      const double gap = separation(boxes[left], boxes[right]);
      if (gap <= 1.0e-12) {
        ++overlapping_pairs;
        join(left, right);
      } else {
        minimum_positive_pair_gap = std::min(
          minimum_positive_pair_gap, gap);
        if (gap < 0.10) {
          ++positive_gap_below_010;
        }
        if (gap < 0.50) {
          ++positive_gap_below_050;
        }
        if (gap < 0.90) {
          ++positive_gap_below_090;
        }
      }
    }
  }

  int connected_components = 0;
  for (int index = 0; index < obstacle_count; ++index) {
    if (root(index) == index) {
      ++connected_components;
    }
  }

  double minimum_component_gap = INFINITY;
  for (int left = 0; left < obstacle_count; ++left) {
    for (int right = left + 1; right < obstacle_count; ++right) {
      if (root(left) == root(right)) {
        continue;
      }
      minimum_component_gap = std::min(
        minimum_component_gap, separation(boxes[left], boxes[right]));
    }
  }

  int boxes_outside_nominal_bounds = 0;
  for (const auto & box : boxes) {
    const double half = 0.5 * box.width;
    if (box.x - half < -20.0 || box.x + half > 20.0 ||
      box.y - half < -20.0 || box.y + half > 20.0)
    {
      ++boxes_outside_nominal_bounds;
    }
  }

  std::cout << std::fixed << std::setprecision(9)
            << "obstacle_count=" << obstacle_count << '\n'
            << "overlapping_pairs=" << overlapping_pairs << '\n'
            << "connected_components=" << connected_components << '\n'
            << "mean_width_m=" << mean_width << '\n'
            << "primitive_area_sum_m2=" << primitive_area << '\n'
            << "minimum_positive_pair_gap_m="
            << minimum_positive_pair_gap << '\n'
            << "minimum_component_gap_m=" << minimum_component_gap << '\n'
            << "positive_pair_gaps_below_0p10="
            << positive_gap_below_010 << '\n'
            << "positive_pair_gaps_below_0p50="
            << positive_gap_below_050 << '\n'
            << "positive_pair_gaps_below_0p90="
            << positive_gap_below_090 << '\n'
            << "boxes_crossing_nominal_boundary="
            << boxes_outside_nominal_bounds << '\n';
}
