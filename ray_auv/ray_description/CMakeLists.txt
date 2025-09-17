cmake_minimum_required(VERSION 3.5)
project(ray_description)

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY 
    launch 
    meshes 
    urdf
    models
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/ray_pressure_sensor.py
  scripts/test_ray_pressure_sensor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_environment_hooks("hooks/resource_paths.dsv.in")
install(DIRECTORY launch meshes urdf DESTINATION share/${PROJECT_NAME})

ament_package()
