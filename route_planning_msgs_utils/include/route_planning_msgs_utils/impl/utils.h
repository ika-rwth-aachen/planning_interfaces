#pragma once

#include <cmath>

namespace route_planning_msgs {

namespace route_access {

/**
 * @brief Wraps an angle to the range [-π, π].
 * 
 * @param angle The input angle in radians.
 * @return The wrapped angle in radians.
 */
inline double wrapAngle(const double& angle) {
  double wrapped_angle = angle;
  while (wrapped_angle > M_PI) wrapped_angle -= 2 * M_PI;
  while (wrapped_angle < -M_PI) wrapped_angle += 2 * M_PI;
  return wrapped_angle;
}

}  // namespace route_access

}  // namespace route_planning_msgs