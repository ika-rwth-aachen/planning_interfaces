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

#include <iostream>

#include <trajectory_planning_msgs_utils/impl/checks.h>
#include <trajectory_planning_msgs_utils/impl/state_getters.h>
#include <trajectory_planning_msgs_utils/impl/state_index.h>
#include <trajectory_planning_msgs_utils/impl/utils.h>

namespace trajectory_planning_msgs {

namespace trajectory_access {

inline void setState(std::vector<double>& state, const unsigned char& type_id, const std::vector<double>& val) {
  sanityCheckStateSize(val, type_id);
  state = val;
}

inline void setState(std::vector<double>& states, const unsigned char& type_id, const std::vector<double>& val,
                     const unsigned int samplePoint) {
  sanityCheckStateSize(val, type_id);
  std::vector<double> state = getState(states, type_id, samplePoint);
  setState(state, type_id, val);
  int idx = samplePoint * getStateDim(type_id);
  std::copy(state.begin(), state.end(), states.begin() + idx);
}

inline void setState(Trajectory& trajectory, const std::vector<double>& val, const unsigned int samplePoint) {
  sanityCheckTrajectory(trajectory);
  setState(trajectory.states, trajectory.type_id, val, samplePoint);
}

inline void setStates(std::vector<double>& states, const unsigned char& type_id, const std::vector<double>& val) {
  sanityCheckStatesSize(val, type_id);
  states = val;
}

inline void setStates(Trajectory& trajectory, const unsigned char& type_id, const std::vector<double>& val) {
  setStates(trajectory.states, type_id, val);
}

inline void setT(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexT(type_id);
  state[idx] = val;
}

inline void setT(std::vector<double>& states, const unsigned char& type_id, const double val, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setT(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setT(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setT(trajectory.states, trajectory.type_id, val, i);
}

inline void setX(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexX(type_id);
  state[idx] = val;
}

inline void setX(std::vector<double>& states, const unsigned char& type_id, const double val, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setX(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setX(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setX(trajectory.states, trajectory.type_id, val, i);
}

inline void setY(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexY(type_id);
  state[idx] = val;
}

inline void setY(std::vector<double>& states, const unsigned char& type_id, const double val, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setY(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setY(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setY(trajectory.states, trajectory.type_id, val, i);
}

inline void setV(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexV(type_id);
  state[idx] = val;
}

inline void setV(std::vector<double>& states, const unsigned char& type_id, const double val, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setV(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setV(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setV(trajectory.states, trajectory.type_id, val, i);
}

inline void setTheta(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexTheta(type_id);
  try {
    sanityCheckAngle(val);
    state[idx] = val;
  } catch (const std::invalid_argument& e) {
    std::cout << "Invalid angle value: " << val << ", wrapping to [-π, π]" << std::endl;
    state[idx] = wrapAngle(val);
  }
}

inline void setTheta(std::vector<double>& states, const unsigned char& type_id, const double val,
                     const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setTheta(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setTheta(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setTheta(trajectory.states, trajectory.type_id, val, i);
}

inline void setA(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexA(type_id);
  state[idx] = val;
}

inline void setA(std::vector<double>& states, const unsigned char& type_id, const double val, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setA(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setA(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setA(trajectory.states, trajectory.type_id, val, i);
}

inline double computeBeta(const double delta_front, const double delta_rear,
                          const double distance_front_axle, const double distance_rear_axle){
  const double wheelbase = distance_front_axle + distance_rear_axle;
  return atan((distance_rear_axle / wheelbase) * tan(delta_front) + (distance_front_axle / wheelbase) * tan(delta_rear));
}

inline void setBeta(std::vector<double>& state, const unsigned char& type_id,
                    const double delta_front, const double delta_rear, const double distance_front_axle,
                    const double distance_rear_axle) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexBeta(type_id);
  double beta = computeBeta(delta_front, delta_rear, distance_front_axle, distance_rear_axle);
  try {
    sanityCheckAngle(beta);
    state[idx] = beta;
  } catch (const std::invalid_argument& e) {
    std::cout << "Invalid angle value: " << beta << ", wrapping to [-π, π]" << std::endl;
    state[idx] = wrapAngle(beta);
  }
}

inline void setBeta(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexBeta(type_id);
  try {
    sanityCheckAngle(val);
    state[idx] = val;
  } catch (const std::invalid_argument& e) {
    std::cout << "Invalid angle value: " << val << ", wrapping to [-π, π]" << std::endl;
    state[idx] = wrapAngle(val);
  }
}

inline void setBeta(std::vector<double>& states, const unsigned char& type_id,
                    const double delta_front, const double delta_rear, const double distance_front_axle,
                    const double distance_rear_axle, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setBeta(state, type_id, delta_front, delta_rear, distance_front_axle, distance_rear_axle);
  setState(states, type_id, state, i);
}

inline void setBeta(std::vector<double>& states, const unsigned char& type_id, const double val, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setBeta(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setBeta(Trajectory& trajectory,
                    const double delta_front, const double delta_rear, const double distance_front_axle,
                    const double distance_rear_axle, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setBeta(trajectory.states, trajectory.type_id, delta_front, delta_rear, distance_front_axle, distance_rear_axle, i);
}

inline void setBeta(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setBeta(trajectory.states, trajectory.type_id, val, i);
}

inline void setDeltaFront(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexDeltaFront(type_id);
  try {
    sanityCheckAngle(val);
    state[idx] = val;
  } catch (const std::invalid_argument& e) {
    std::cout << "Invalid angle value: " << val << ", wrapping to [-π, π]" << std::endl;
    state[idx] = wrapAngle(val);
  }
}

inline void setDeltaFront(std::vector<double>& states, const unsigned char& type_id, const double val,
                       const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setDeltaFront(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setDeltaFront(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setDeltaFront(trajectory.states, trajectory.type_id, val, i);
}

inline void setDeltaRear(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexDeltaRear(type_id);
  try {
    sanityCheckAngle(val);
    state[idx] = val;
  } catch (const std::invalid_argument& e) {
    std::cout << "Invalid angle value: " << val << ", wrapping to [-π, π]" << std::endl;
    state[idx] = wrapAngle(val);
  }
}

inline void setDeltaRear(std::vector<double>& states, const unsigned char& type_id, const double val,
                       const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setDeltaRear(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setDeltaRear(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setDeltaRear(trajectory.states, trajectory.type_id, val, i);
}

inline void setDeltaAck(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexDeltaAck(type_id);
  try {
    sanityCheckAngle(val);
    state[idx] = val;
  } catch (const std::invalid_argument& e) {
    std::cout << "Invalid angle value: " << val << ", wrapping to [-π, π]" << std::endl;
    state[idx] = wrapAngle(val);
  }
}

inline void setDeltaAck(std::vector<double>& states, const unsigned char& type_id, const double val,
                     const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setDeltaAck(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setDeltaAck(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setDeltaAck(trajectory.states, trajectory.type_id, val, i);
}

inline void setS(std::vector<double>& state, const unsigned char& type_id, const double val) {
  sanityCheckStateSize(state, type_id);
  const int idx = indexS(type_id);
  state[idx] = val;
}

inline void setS(std::vector<double>& states, const unsigned char& type_id, const double val, const unsigned int i) {
  sanityCheckStatesSize(states, type_id);
  std::vector<double> state = getState(states, type_id, i);
  setS(state, type_id, val);
  setState(states, type_id, state, i);
}

inline void setS(Trajectory& trajectory, const double val, const unsigned int i) {
  sanityCheckTrajectory(trajectory);
  setS(trajectory.states, trajectory.type_id, val, i);
}

inline void setStandstill(Trajectory& trajectory, const bool val) {
  sanityCheckTrajectory(trajectory);
  trajectory.standstill = val;
}

}  // namespace trajectory_access

}  // namespace trajectory_planning_msgs