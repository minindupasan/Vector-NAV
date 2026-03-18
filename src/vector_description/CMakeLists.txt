cmake_minimum_required(VERSION 3.8)
project(vector_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes launch config worlds rviz models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
