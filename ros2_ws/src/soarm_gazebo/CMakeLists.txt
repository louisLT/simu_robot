cmake_minimum_required(VERSION 3.8)
project(soarm_gazebo)

find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY worlds models config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
