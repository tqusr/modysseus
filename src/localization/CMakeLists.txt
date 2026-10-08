cmake_minimum_required(VERSION 3.8)
project(localization)

find_package(ament_cmake REQUIRED)
# Stays ament_cmake because it owns launch files, configs and saved maps.
# ament_cmake_python is only so the ROS-free maths in localization/ imports (and
# is unit-tested) as a module. The nodes stay in scripts/ as plain programs.
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
  # Pure Python: no ROS graph, no robot.
  ament_add_pytest_test(test_wheel_odometry test/test_wheel_odometry.py)
  ament_add_pytest_test(test_gyro_bias test/test_gyro_bias.py)
  ament_add_pytest_test(test_wheel_drive test/test_wheel_drive.py)
endif()

ament_package()
