#pragma once

// Preserve the original trajectory coefficient ordering and evaluation
// semantics from the in-tree copied traj_utils implementation.
#include <traj_utils/polynomial_traj.h>

namespace m20_trajectory {
using PolynomialTraj = ::PolynomialTraj;
}
