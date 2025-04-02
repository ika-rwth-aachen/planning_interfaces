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

#include <trajectory_planning_msgs_utils/impl/checks.h>
#include <trajectory_planning_msgs_utils/impl/state_index.h>
#include <trajectory_planning_msgs_utils/impl/utils.h>

namespace trajectory_planning_msgs {

namespace trajectory_access {

inline int getSamplePointSize(const std::vector<double>& states, const unsigned char& type_id) {
  sanityCheckStatesSize(states, type_id);
  int states_size = getStatesSize(states);
  int state_dim = getStateDim(type_id);
  return states_size / state_dim;
}

inline int getSamplePointSize(const Trajectory& trajectory) {
  return getSamplePointSize(trajectory.states, trajectory.type_id);
}

inline std::vector<double> getState(const std::vector<double>& states, const unsigned char& type_id,
                                    const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  unsigned int sample_point_size = getSamplePointSize(states, type_id);
  unsigned int start = i * getStateDim(type_id);
  unsigned int end = (i + 1) * getStateDim(type_id);
  if (i >= sample_point_size || end > getStatesSize(states))
    throw std::out_of_range("Index i = " + std::to_string(i) + " out of range (" + std::to_string(sample_point_size) +
                            ")");

  return std::vector<double>(states.begin() + start, states.begin() + end);
}

inline std::vector<double> getState(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getState(trajectory.states, trajectory.type_id, i);
}

inline double getT(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  return state[indexT(type_id)];
}

inline double getT(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getT(state, type_id);
}

inline double getT(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getT(trajectory.states, trajectory.type_id, i);
}

inline double getX(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  return state[indexX(type_id)];
}

inline double getX(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getX(state, type_id);
}

inline double getX(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getX(trajectory.states, trajectory.type_id, i);
}

inline double getY(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  return state[indexY(type_id)];
}

inline double getY(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getY(state, type_id);
}

inline double getY(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getY(trajectory.states, trajectory.type_id, i);
}

inline double getV(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  return state[indexV(type_id)];
}

inline double getV(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getV(state, type_id);
}

inline double getV(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getV(trajectory.states, trajectory.type_id, i);
}

inline double approximateTheta(const std::vector<double>& state1, const std::vector<double>& state2,
                               const unsigned char& type_id) {
  double dx = getX(state2, type_id) - getX(state1, type_id);
  double dy = getY(state2, type_id) - getY(state1, type_id);
  return std::atan2(dy, dx);
}

inline double getTheta(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  double theta = state[indexTheta(type_id)];
  sanityCheckAngle(theta);
  return theta;
}

inline double getTheta(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  if (type_id == DRIVABLE::TYPE_ID || type_id == DRIVABLERWS::TYPE_ID) {
    std::vector<double> state = getState(states, type_id, i);
    return getTheta(state, type_id);
  } else if (type_id == REFERENCE::TYPE_ID) {
    unsigned int nSamples = getSamplePointSize(states, type_id);
    std::vector<double> state1, state2;
    if (i == 0) {
      state1 = getState(states, type_id, i);
      state2 = getState(states, type_id, i + 1);
    } else if (i > 0 && i < nSamples - 1) {
      state1 = getState(states, type_id, i - 1);
      state2 = getState(states, type_id, i + 1);
    } else if (i == nSamples - 1) { 
      state1 = getState(states, type_id, i - 1);
      state2 = getState(states, type_id, i);
    }
    double theta = approximateTheta(state1, state2, type_id);
    sanityCheckAngle(theta);
    return theta;
  } else {
    throw std::invalid_argument(kExceptionUnknownStateEntry + std::to_string(type_id) + ", " + "theta");
  }
}

inline double getTheta(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getTheta(trajectory.states, trajectory.type_id, i);
}

inline double getA(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  return state[indexA(type_id)];
}

inline double getA(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getA(state, type_id);
}

inline double getA(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getA(trajectory.states, trajectory.type_id, i);
}

inline double getBeta(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  double beta = state[indexBeta(type_id)];
  sanityCheckAngle(beta);
  return beta;
}

inline double getBeta(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getBeta(state, type_id);
}

inline double getBeta(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getBeta(trajectory.states, trajectory.type_id, i);
}

inline double getDeltaFront(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  double delta_front = state[indexDeltaFront(type_id)];
  sanityCheckAngle(delta_front);
  return delta_front;
}

inline double getDeltaFront(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getDeltaFront(state, type_id);
}

inline double getDeltaFront(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getDeltaFront(trajectory.states, trajectory.type_id, i);
}

inline double getDeltaRear(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  double delta_rear = state[indexDeltaRear(type_id)];
  sanityCheckAngle(delta_rear);
  return delta_rear;
}

inline double getDeltaRear(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getDeltaRear(state, type_id);
}

inline double getDeltaRear(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getDeltaRear(trajectory.states, trajectory.type_id, i);
}

inline double getDeltaAck(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  double delta_ack = state[indexDeltaAck(type_id)];
  sanityCheckAngle(delta_ack);
  return delta_ack;
}

inline double getDeltaAck(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getDeltaAck(state, type_id);
}

inline double getDeltaAck(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getDeltaAck(trajectory.states, trajectory.type_id, i);
}

inline double getS(const std::vector<double>& state, const unsigned char& type_id) {
  sanityCheckStateSize(state, type_id);
  return state[indexS(type_id)];
}

inline double getS(const std::vector<double>& states, const unsigned char& type_id, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  return getS(state, type_id);
}

inline double getS(const Trajectory& trajectory, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  return getS(trajectory.states, trajectory.type_id, i);
}

inline bool getStandstill(const Trajectory& trajectory) {
  sanityCheckTrajectory(trajectory);
  return trajectory.standstill;
}

}  // namespace trajectory_access

}  // namespace trajectory_planning_msgs