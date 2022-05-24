cmake_minimum_required(VERSION 3.0.2)
project(pizza_stage1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      tf2
      moveit_core
      moveit_ros_planning
      moveit_ros_planning_interface
      moveit_ros_perception
)

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(stage1 src/stage1.cpp)
add_dependencies(stage1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(stage1
  ${catkin_LIBRARIES}
)
