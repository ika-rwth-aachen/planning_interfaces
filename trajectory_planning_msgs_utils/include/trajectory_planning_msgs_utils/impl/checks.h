// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

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