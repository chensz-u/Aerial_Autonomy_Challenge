cmake_minimum_required(VERSION 3.0.2)
project(c5_autonomous_exploration)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  pcl_conversions
  pcl_ros
  quadrotor_msgs
  roscpp
  sensor_msgs
  std_msgs
)

find_package(PCL REQUIRED)

add_message_files(
  FILES
  ExplorationStatus.msg
  PlannerFeedback.msg
  PlannerRequest.msg
  TargetTrack.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES c5_autonomous_exploration_core
  CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs pcl_conversions pcl_ros quadrotor_msgs roscpp sensor_msgs std_msgs
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(c5_autonomous_exploration_core
  src/autonomous_exploration.cpp
  src/dynamic_scene_filter.cpp
  src/eskf.cpp
  src/local_map.cpp
  src/loop_closure.cpp
  src/mission_executor.cpp
  src/model_predictive_controller.cpp
  src/runtime_support.cpp
  src/semantic_target_processor.cpp
  src/state_estimator.cpp
  src/target_evidence.cpp
  src/vio_frontend.cpp
)

add_dependencies(c5_autonomous_exploration_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(c5_autonomous_exploration_core ${catkin_LIBRARIES})

add_executable(c5_autonomous_exploration_node
  src/c5_autonomous_exploration_node.cpp
)

add_dependencies(c5_autonomous_exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(c5_autonomous_exploration_node c5_autonomous_exploration_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(c5_autonomous_exploration_core_test test/test_autonomous_exploration.cpp)
  target_link_libraries(c5_autonomous_exploration_core_test c5_autonomous_exploration_core ${catkin_LIBRARIES})
  add_test(NAME c5_autonomous_exploration_core_test COMMAND c5_autonomous_exploration_core_test)

  add_executable(c5_dynamic_scene_filter_test test/test_dynamic_scene_filter.cpp)
  target_link_libraries(c5_dynamic_scene_filter_test c5_autonomous_exploration_core )
  add_test(NAME c5_dynamic_scene_filter_test COMMAND c5_dynamic_scene_filter_test)

  add_executable(c5_frontier_performance_test test/test_frontier_performance.cpp)
  target_link_libraries(c5_frontier_performance_test c5_autonomous_exploration_core ${catkin_LIBRARIES})
  add_test(NAME c5_frontier_performance_test COMMAND c5_frontier_performance_test)
endif()
