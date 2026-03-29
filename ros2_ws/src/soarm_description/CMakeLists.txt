cmake_minimum_required(VERSION 3.8)
project(soarm_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY urdf meshes config rviz launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
