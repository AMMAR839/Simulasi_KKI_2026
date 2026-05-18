cmake_minimum_required(VERSION 3.8)
project(asv_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY models scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
