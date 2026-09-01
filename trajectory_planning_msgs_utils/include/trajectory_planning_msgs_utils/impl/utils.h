// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <cmath>

namespace trajectory_planning_msgs {

namespace trajectory_access {

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

// --- state size ------------------------------------------------------------

const std::string kExceptionUnknownType = "Unknown type ID: ";

inline int getStateDim(const unsigned char& type_id) {
  switch (type_id) {
    case DRIVABLE::TYPE_ID:
      return DRIVABLE::STATE_DIM;
    case DRIVABLERWS::TYPE_ID:
      return DRIVABLERWS::STATE_DIM;
    case REFERENCE::TYPE_ID:
      return REFERENCE::STATE_DIM;
    default:
      throw std::invalid_argument(kExceptionUnknownType + std::to_string(type_id));
  }
}

inline int getStateDim(const Trajectory& trajectory) { return getStateDim(trajectory.type_id); }

inline unsigned int getStatesSize(const std::vector<double>& states) { return states.size(); }

inline unsigned int getStatesSize(const Trajectory& trajectory) { return getStatesSize(trajectory.states); }

inline bool isFloatingDivision(int numerator, int denominator) { return numerator % denominator != 0; }

}  // namespace trajectory_access

}  // namespace trajectory_planning_msgs