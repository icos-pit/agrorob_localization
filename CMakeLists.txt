cmake_minimum_required(VERSION 3.8)
project(farmbot_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

cmake_policy(SET CMP0074 NEW)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(farmbot_interfaces REQUIRED)

add_executable(antenna_fuse src/antenna_fuse.cpp)
ament_target_dependencies(antenna_fuse
  rclcpp 
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_ros
  pcl_conversions
  farmbot_interfaces
)

add_executable(antenna_split src/antenna_split.cpp)
ament_target_dependencies(antenna_split
  rclcpp 
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_ros
  pcl_conversions
  farmbot_interfaces
)

add_executable(gps_to_enu src/gps_to_enu.cpp)
ament_target_dependencies(gps_to_enu
  rclcpp 
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_ros
  pcl_conversions
  farmbot_interfaces
)

add_executable(odom_n_path src/odom_n_path.cpp)
ament_target_dependencies(odom_n_path
  rclcpp 
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_ros
  pcl_conversions
  farmbot_interfaces
)

add_executable(transform_pub src/transform_pub.cpp)
ament_target_dependencies(transform_pub
  rclcpp 
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_ros
  pcl_conversions
  farmbot_interfaces
)


find_package(Eigen3 REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})
link_directories(${Eigen_LIBRARY_DIRS})
add_definitions(${Eigen_DEFINITIONS})
find_package(fmt REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

target_link_libraries(antenna_fuse
  fmt::fmt
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
)
target_link_libraries(antenna_split
  fmt::fmt
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
)
target_link_libraries(gps_to_enu
  fmt::fmt
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
)
target_link_libraries(odom_n_path
  fmt::fmt
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
)
target_link_libraries(transform_pub
  fmt::fmt
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
)

install(TARGETS
  antenna_fuse antenna_split gps_to_enu odom_n_path transform_pub
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
