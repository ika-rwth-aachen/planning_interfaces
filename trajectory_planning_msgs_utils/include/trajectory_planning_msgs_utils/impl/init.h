// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

#include <trajectory_planning_msgs_utils/impl/state_setters.h>
#include <trajectory_planning_msgs_utils/impl/utils.h>

namespace trajectory_planning_msgs {

namespace trajectory_access {

const double STATE_INIT = 0;

inline void initializeStates(std::vector<double>& states, const unsigned char& type_id,
                             const unsigned int nSamplePoints) {
  int vector_size = nSamplePoints * getStateDim(type_id);
  setStates(states, type_id, std::vector<double>(vector_size, STATE_INIT));
}

inline void initializeTrajectory(Trajectory& trajectory, const unsigned char& type_id,
                                 const unsigned int nSamplePoints) {
  initializeStates(trajectory.states, type_id, nSamplePoints);
  trajectory.type_id = type_id;
  trajectory.standstill = true;
}

}  // namespace trajectory_access

}  // namespace trajectory_planning_msgs