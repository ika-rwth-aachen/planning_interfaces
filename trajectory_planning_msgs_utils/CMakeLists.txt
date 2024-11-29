cmake_minimum_required(VERSION 3.8)
project(trajectory_planning_msgs_utils)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})
set(ROS_DISTRO $ENV{ROS_DISTRO})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(trajectory_planning_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  add_library(${PROJECT_NAME} INTERFACE)

  target_include_directories(${PROJECT_NAME} INTERFACE
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  target_link_libraries(${PROJECT_NAME} INTERFACE
    ${std_msgs_TARGETS}
    ${trajectory_planning_msgs_TARGETS}
  )

  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)

  ament_export_dependencies(
    trajectory_planning_msgs
    std_msgs
  )

  install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
  )

  install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
    INCLUDES DESTINATION include
  )

  if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}-test test/test_trajectory_access.ros2.cpp)
    target_include_directories(${PROJECT_NAME}-test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_include_directories(${PROJECT_NAME}-test PUBLIC test)
    ament_target_dependencies(${PROJECT_NAME}-test
      std_msgs
      trajectory_planning_msgs
    )
  endif()

  ament_package()

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED COMPONENTS
    std_msgs
    trajectory_planning_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      std_msgs
      trajectory_planning_msgs
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory_access.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_include_directories(${PROJECT_NAME}-test PUBLIC test)
  endif()

endif()