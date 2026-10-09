cmake_minimum_required(VERSION 3.8)
project(l2_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/l2_initial_pose.py scripts/l2_data_watch.py scripts/l2_wait_for_data.py DESTINATION lib/${PROJECT_NAME})

ament_package()
