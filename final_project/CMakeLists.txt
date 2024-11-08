cmake_minimum_required(VERSION 3.8)
project(final_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add pkg-config support
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED IMPORTED_TARGET zbar)

# Find other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)  # Add this for Twist messages

# Add MoveIt dependencies
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(gopro_publisher src/gopro_publisher.cpp)

target_link_libraries(gopro_publisher
  ${OpenCV_LIBS}
  PkgConfig::ZBAR
)

ament_target_dependencies(gopro_publisher 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport 
  OpenCV 
  camera_info_manager
  std_msgs
  geometry_msgs  # Add this for Twist messages
)

# Install C++ node
install(TARGETS
  gopro_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    scripts/visual_controller.py
    scripts/control_command.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
