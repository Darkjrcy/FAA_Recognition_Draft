cmake_minimum_required(VERSION 3.5)
project(gnss_multipath_plugin)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_plugins REQUIRED)
find_package(PkgConfig REQUIRED)
set(ENV{PKG_CONFIG_PATH} "${CMAKE_INSTALL_PREFIX}/../libpredict/lib/pkgconfig/\
:/usr/share/pkg-config\
:/usr/lib/pkg-config
:$ENV{PKG_CONFIG_PATH}")
pkg_check_modules(PREDICT REQUIRED IMPORTED_TARGET predict)
include_directories(include ${GAZEBO_INCLUDE_DIRS})
add_library(GNSSMultipathSensorPlugin SHARED src/multipath_sensor.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GNSSMultipathFix.msg"
  DEPENDENCIES std_msgs sensor_msgs builtin_interfaces
)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
target_include_directories(GNSSMultipathSensorPlugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> ${PREDICT_INCLUDE_DIRS} ${PREDICT_LIBRARY_DIRS})
target_link_libraries(GNSSMultipathSensorPlugin ${GAZEBO_LIBRARIES} ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES}  ${PREDICT_LINK_LIBRARIES})
ament_target_dependencies(GNSSMultipathSensorPlugin
 "gazebo_ros"
 "rclcpp"
 "sensor_msgs"
 "std_msgs")

 rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
 target_link_libraries(GNSSMultipathSensorPlugin "${cpp_typesupport_target}")
install(TARGETS GNSSMultipathSensorPlugin
  DESTINATION lib)
install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)  
install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/
) 
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
) 
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
) 
install(DIRECTORY
  data
  DESTINATION lib/
) 
ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()