cmake_minimum_required(VERSION 3.8)
project(custom_thruster_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_components REQUIRED)

# Include directories
include_directories(include)

# Create the library
add_library(${PROJECT_NAME} SHARED
  src/custom_thruster_manager.cpp
)

# Create the executable
add_executable(${PROJECT_NAME}_node
  src/custom_thruster_manager_node.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  rclcpp_components
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Link against Eigen
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
target_link_libraries(${PROJECT_NAME}_node Eigen3::Eigen ${PROJECT_NAME})

# Add compile definitions for Eigen
target_compile_definitions(${PROJECT_NAME} PRIVATE EIGEN_MPL2_ONLY)
target_compile_definitions(${PROJECT_NAME}_node PRIVATE EIGEN_MPL2_ONLY)

# Register the component (commented out since we're using standalone node)
# rclcpp_components_register_nodes(${PROJECT_NAME}
#   "custom_thruster_manager::CustomThrusterManager"
# )

# Install the library and executable
install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(TARGETS
  ${PROJECT_NAME}_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
