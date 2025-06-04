cmake_minimum_required(VERSION 3.8)
project(airplane_gazebo_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(${gazebo_dev_LIBRARY_DIRS})

# Build the airplane_3D_movement plugin
add_library(airplane_3d_movement_gazebo_plugin SHARED
  src/airplane_3d_movement_gazebo_plugin.cpp)
target_include_directories(airplane_3d_movement_gazebo_plugin PUBLIC include)

ament_target_dependencies(airplane_3d_movement_gazebo_plugin
  "gazebo_dev"
  "gazebo_ros"
  "nav_msgs"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
  "tf2_ros"
)

# Build the real-time factor plugin
add_library(real_time_factor_plugin SHARED src/real_time_factor_publisher.cpp)
ament_target_dependencies(real_time_factor_plugin
  "gazebo_ros"
  "rclcpp" 
  "std_msgs" # Add dependency on the message package
)

# Install targets
install(TARGETS
  airplane_3d_movement_gazebo_plugin
  real_time_factor_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directories
install(DIRECTORY include/
  DESTINATION include
)

# Install share directories
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()