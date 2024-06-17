cmake_minimum_required(VERSION 3.5)
project(turtlebot3_support)


find_package(ament_cmake REQUIRED)
find_package(joy REQUIRED)
find_package(teleop_twist_joy REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    joy
    teleop_twist_joy
)

install(DIRECTORY launch script DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
