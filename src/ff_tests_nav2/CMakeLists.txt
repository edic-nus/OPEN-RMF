cmake_minimum_required(VERSION 3.5)

project(ff_tests_nav2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params rviz_configs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()