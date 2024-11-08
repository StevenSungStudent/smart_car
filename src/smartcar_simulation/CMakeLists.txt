cmake_minimum_required(VERSION 3.8)
project(smartcar_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(smartcar_msgs)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Add the Odometry executable
add_executable(Odometry 
  src/OdometryMain.cpp 
  src/Odometry.cpp
)

ament_target_dependencies(Odometry
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  smartcar_msgs
)

# Add the JointStatePublisher executable
add_executable(JointStatePublisher
  src/JointStatePublisherMain.cpp 
  src/JointStatePublisher.cpp 
)

ament_target_dependencies(JointStatePublisher
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
  smartcar_msgs
  sensor_msgs
)

# Include directories for both executables
target_include_directories(Odometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)

target_include_directories(JointStatePublisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
  $<INSTALL_INTERFACE:include>
)


install( DIRECTORY launch rviz urdf world map include config
  DESTINATION share/${PROJECT_NAME})

  # Install both executables
install(TARGETS Odometry
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS JointStatePublisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
