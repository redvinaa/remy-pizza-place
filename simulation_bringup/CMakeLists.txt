cmake_minimum_required(VERSION 3.0.2)
project(simulation_bringup)

find_package(catkin REQUIRED COMPONENTS
  franka_gripper
  gazebo_ros
)

catkin_package()
