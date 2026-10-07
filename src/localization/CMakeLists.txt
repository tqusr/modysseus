cmake_minimum_required(VERSION 3.8)
project(localization)

find_package(ament_cmake REQUIRED)
# This stays an ament_cmake package - it owns launch files, YAML configs and
# the saved maps, none of which belong to a Python package - and gains
# ament_cmake_python only so the ROS-free maths in localization/ can be
# imported (and unit-tested) as a real module. The nodes themselves stay in
# scripts/, installed as plain programs, which is the pattern
# slam_lifecycle_watchdog.py already established here.
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY launch config maps DESTINATION share/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
        scripts/slam_lifecycle_watchdog.py
        scripts/wheel_odometry_node.py
        scripts/gyro_bias_node.py
        scripts/wheel_drive_node.py
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Pure Python against pure Python: no ROS graph, no robot, no Gazebo.
  ament_add_pytest_test(test_wheel_odometry test/test_wheel_odometry.py)
  ament_add_pytest_test(test_gyro_bias test/test_gyro_bias.py)
  ament_add_pytest_test(test_wheel_drive test/test_wheel_drive.py)
endif()

ament_package()
