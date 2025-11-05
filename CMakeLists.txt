cmake_minimum_required(VERSION 3.8)
project(fusion_lidar_camera)

# Set CMake policies
if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yolo_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

# Create executables
add_executable(lidar_camera_fusion_node src/main.cpp src/fusion_node.cpp)
add_executable(simple_lidar_camera_fusion_node src/simple_main.cpp)
add_executable(test_lidar_camera_fusion_node src/test_main.cpp)


# Target dependencies
ament_target_dependencies(lidar_camera_fusion_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  pcl_conversions
  tf2_eigen
  tf2_geometry_msgs
  yolo_msgs
)

ament_target_dependencies(simple_lidar_camera_fusion_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  pcl_conversions
)

ament_target_dependencies(test_lidar_camera_fusion_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  pcl_conversions
)

# Link libraries
target_link_libraries(lidar_camera_fusion_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

target_link_libraries(simple_lidar_camera_fusion_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

target_link_libraries(test_lidar_camera_fusion_node
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

# Install executables
install(TARGETS
  lidar_camera_fusion_node
  simple_lidar_camera_fusion_node
  test_lidar_camera_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  OPTIONAL
)

# Install config files
install(DIRECTORY
  config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
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
