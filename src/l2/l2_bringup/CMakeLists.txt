cmake_minimum_required(VERSION 3.8)
project(l2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/l2_initial_pose.py DESTINATION lib/${PROJECT_NAME})

ament_package()
