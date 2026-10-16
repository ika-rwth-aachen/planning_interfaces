// Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
// SPDX-License-Identifier: MIT

#pragma once

// Messages
#include <trajectory_planning_msgs/msg/trajectory.hpp>

// Trajectory Types
#include <trajectory_planning_msgs/msg/drivable.hpp>
#include <trajectory_planning_msgs/msg/drivablerws.hpp>
#include <trajectory_planning_msgs/msg/reference.hpp>

namespace trajectory_planning_msgs {
using namespace msg;
}

#include <trajectory_planning_msgs_utils/checks.hpp>
#include <trajectory_planning_msgs_utils/init.hpp>
#include <trajectory_planning_msgs_utils/state_getters.hpp>
#include <trajectory_planning_msgs_utils/state_index.hpp>
#include <trajectory_planning_msgs_utils/state_setters.hpp>
#include <trajectory_planning_msgs_utils/utils.hpp>