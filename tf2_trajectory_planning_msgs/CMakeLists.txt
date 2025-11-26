cmake_minimum_required(VERSION 3.8)
project(tf2_trajectory_planning_msgs)

find_package(ros_environment REQUIRED QUIET)
set(ROS_VERSION $ENV{ROS_VERSION})

# === ROS2 (AMENT) =============================================================
if(${ROS_VERSION} EQUAL 2)

  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  # find dependencies
  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(trajectory_planning_msgs REQUIRED)
  find_package(trajectory_planning_msgs_utils REQUIRED)

  ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR src/${PROJECT_NAME}
  )

  add_library(${PROJECT_NAME} INTERFACE)

  target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_dependencies(
    geometry_msgs
    rclcpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    trajectory_planning_msgs
    trajectory_planning_msgs_utils
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
    ament_add_gtest(${PROJECT_NAME}-test test/test_tf2_trajectory_planning_msgs.ros2.cpp)
    target_include_directories(${PROJECT_NAME}-test PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_include_directories(${PROJECT_NAME}-test PUBLIC test)
    ament_target_dependencies(${PROJECT_NAME}-test
      geometry_msgs
      rclcpp
      tf2
      tf2_geometry_msgs
      tf2_ros
      trajectory_planning_msgs
      trajectory_planning_msgs_utils
    )

    find_package(ament_cmake_pytest REQUIRED)
    set(_pytest_tests
      test/test_tf2_trajectory_planning_msgs.py
    )
    foreach(_test_path ${_pytest_tests})
      get_filename_component(_test_name ${_test_path} NAME_WE)
      ament_add_pytest_test(${_test_name} ${_test_path}
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 60
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
      )
    endforeach()
  endif()

  ament_package()

# === ROS1 (CATKIN) ============================================================
elseif(${ROS_VERSION} EQUAL 1)

  add_compile_definitions(ROS1)

  find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    roscpp
    tf2
    tf2_geometry_msgs
    trajectory_planning_msgs
    trajectory_planning_msgs_utils
  )

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      geometry_msgs
      roscpp
      tf2
      tf2_geometry_msgs
      trajectory_planning_msgs
      trajectory_planning_msgs_utils
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

  catkin_add_gtest(${PROJECT_NAME}-test test/test_tf2_trajectory_planning_msgs.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_include_directories(${PROJECT_NAME}-test PUBLIC test)
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES})
  endif()

endif()
