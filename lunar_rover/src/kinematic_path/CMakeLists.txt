cmake_minimum_required(VERSION 3.8)
project(kinematic_path)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add message generation
find_package(rosidl_default_generators REQUIRED)

# Specify message files
set(msg_files
  "msg/WheelVelocities.msg"
  DEPENDENCIES std_msgs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

# Include directories
include_directories(include)

# Find all source files
file(GLOB_RECURSE SRC_FILES src/*.cpp)

# Create the executable
add_executable(${PROJECT_NAME}_node ${SRC_FILES})
ament_target_dependencies(${PROJECT_NAME}_node rclcpp std_msgs geometry_msgs rosidl_default_runtime)
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

# Install the executable
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()