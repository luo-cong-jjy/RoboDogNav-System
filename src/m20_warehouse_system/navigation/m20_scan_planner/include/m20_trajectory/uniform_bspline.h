#pragma once

// M20-owned include path for the copied implementation. Preserve the
// original 3xN control-point layout and evaluation semantics.
#include <bspline_opt/uniform_bspline.h>

namespace m20_trajectory {
using UniformBspline = scan_planner::UniformBspline;
}
