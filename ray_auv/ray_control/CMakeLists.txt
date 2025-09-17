cmake_minimum_required(VERSION 3.5)
project(ray_control)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# Install Python scripts
# Note: Pressure sensor scripts moved to ray_description package
  
ament_package()
