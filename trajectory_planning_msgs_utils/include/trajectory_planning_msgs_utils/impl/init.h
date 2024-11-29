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