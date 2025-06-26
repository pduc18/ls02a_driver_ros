cmake_minimum_required(VERSION 3.8)
project(line_laser_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rcpputils REQUIRED)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Declare C++ executables
add_executable(ls02a_serial_publisher
  src/ls02a_lidar.cpp
  src/ls02a_publisher.cpp
)
ament_target_dependencies(ls02a_serial_publisher
  rclcpp
  sensor_msgs
  std_msgs
  Boost
  rcpputils
)
target_link_libraries(ls02a_serial_publisher
  ${Boost_LIBRARIES}
)

add_executable(lidar_processor
  src/lidar_processor.cpp
)
ament_target_dependencies(lidar_processor
  rclcpp
  sensor_msgs
  std_msgs
  Boost
  rcpputils
)
target_link_libraries(lidar_processor
  ${Boost_LIBRARIES}
)

# Install targets
install(TARGETS
  ls02a_serial_publisher
  lidar_processor
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install udev rules
install(FILES scripts/99-laser.rules
  DESTINATION share/${PROJECT_NAME}/udev
)

# Install scripts
install(PROGRAMS
  scripts/create_udev_rules.sh
  scripts/delete_udev_rules.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
