cmake_minimum_required(VERSION 3.8)
project(mapping_docker)

find_package(ament_cmake REQUIRED)

ament_package()


install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/static_relay.py
  DESTINATION lib/${PROJECT_NAME})



