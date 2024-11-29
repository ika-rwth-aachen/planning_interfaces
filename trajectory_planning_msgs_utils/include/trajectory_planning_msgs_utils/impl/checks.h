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

#include <trajectory_planning_msgs_utils/impl/state_index.h>
#include <trajectory_planning_msgs_utils/impl/utils.h>

namespace trajectory_planning_msgs {

namespace trajectory_access {

const std::string kExceptionInvalidStateSize = "Invalid state size for trajectory with type ID: ";
const std::string kExceptionInvalidStatesSize = "Invalid states (sample points) size for trajectory with type ID: ";

inline void sanityCheckStateSize(const std::vector<double>& state, const unsigned char& type_id) {
  int state_dim = getStateDim(type_id);
  int state_size = state.size();
  if (state_size != state_dim)
    throw std::invalid_argument(kExceptionInvalidStateSize + std::to_string(type_id) + ", " +
                                std::to_string(state_size) + " != " + std::to_string(state_dim));
}

inline void sanityCheckStatesSize(const std::vector<double>& states, const unsigned char& type_id) {
  int state_dim = getStateDim(type_id);
  int states_size = getStatesSize(states);
  if (isFloatingDivision(states_size, state_dim))
    throw std::invalid_argument(kExceptionInvalidStatesSize + std::to_string(type_id) + ", states_size = " +
                                std::to_string(states_size) + ", state_dim = " + std::to_string(state_dim));
}

inline void sanityCheckStatesSize(const Trajectory& trajectory) {
  sanityCheckStatesSize(trajectory.states, trajectory.type_id);
}

inline void sanityCheckAngle(const double& angle, const double val_min = -M_PI, const double val_max = M_PI) {
  if (angle < val_min || angle > val_max) {
    throw std::invalid_argument("Angle value: " + std::to_string(angle) + " out of range [" + std::to_string(val_min) +
                                ", " + std::to_string(val_max) + "]");
  }
}

inline void sanityCheckTrajectory(const Trajectory& trajectory) {
  sanityCheckStatesSize(trajectory);
  // ToDo: more checks
}

}  // namespace trajectory_access

}  // namespace trajectory_planning_msgs