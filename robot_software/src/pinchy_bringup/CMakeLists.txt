cmake_minimum_required(VERSION 3.8)
project(pinchy_bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/mock_trajectory.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/joint_sliders.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/ros_bench_test.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/startup_ready.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/motion_monitor.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/start_arm DESTINATION lib/${PROJECT_NAME})

ament_package()
