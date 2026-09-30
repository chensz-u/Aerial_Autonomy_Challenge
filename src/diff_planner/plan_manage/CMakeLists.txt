cmake_minimum_required(VERSION 2.8.3)
project(diff_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  c5_autonomous_exploration
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES diff_planner
  CATKIN_DEPENDS c5_autonomous_exploration geometry_msgs nav_msgs pcl_conversions pcl_ros plan_env path_searching quadrotor_msgs sensor_msgs traj_opt traj_utils
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(diff_planner_node
  src/diff_planner_node.cpp 
  src/diff_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(diff_planner_node 
  ${catkin_LIBRARIES}
  )
#add_dependencies(diff_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})

add_library(c5_planner_bridge_core src/c5_planner_bridge_core.cpp)
target_link_libraries(c5_planner_bridge_core ${catkin_LIBRARIES})

add_executable(c5_planner_bridge_node src/c5_planner_bridge_node.cpp)
add_dependencies(c5_planner_bridge_node ${c5_autonomous_exploration_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(c5_planner_bridge_node c5_planner_bridge_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(c5_planner_bridge_core_test test/c5_planner_bridge_core_test.cpp)
  target_link_libraries(c5_planner_bridge_core_test c5_planner_bridge_core)
  add_test(NAME c5_planner_bridge_core_test COMMAND c5_planner_bridge_core_test)
endif()
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_executable(monitor_node src/monitor_node.cpp)
# target_link_libraries(monitor_node ${catkin_LIBRARIES})

