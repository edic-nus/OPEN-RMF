cmake_minimum_required(VERSION 3.5)

project(ff_tests)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  fleet_configs/
  DESTINATION share/${PROJECT_NAME}/fleet_configs
)

install(DIRECTORY
  zenoh_configs/
  DESTINATION share/${PROJECT_NAME}/zenoh_configs
)

ament_package()
