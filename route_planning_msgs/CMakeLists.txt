cmake_minimum_required(VERSION 3.8)
project(route_planning_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "action/GlobalManeuver.action"
    "msg/DrivableSpaceElement.msg"
    "msg/LaneElement.msg"
    "msg/LaneSeparatorType.msg"
    "msg/RouteElement.msg"
    "msg/RegulatoryElement.msg"
    "msg/RegulatoryElementType.msg"
    "msg/Route.msg"
    DEPENDENCIES std_msgs builtin_interfaces geometry_msgs
  )

  ament_export_dependencies(
    builtin_interfaces
    geometry_msgs
    rosidl_default_runtime
    std_msgs
  )

  ament_package()

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
  )

  add_message_files(DIRECTORY msg)

  generate_messages(
    DEPENDENCIES
      geometry_msgs
      std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS
      geometry_msgs
      message_runtime
      std_msgs
  )

endif()