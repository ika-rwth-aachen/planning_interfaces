/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

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