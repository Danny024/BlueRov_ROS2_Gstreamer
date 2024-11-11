cmake_minimum_required(VERSION 3.8)
project(bluerov_driver)

# Required C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(mavros REQUIRED)  # This includes libmavconn
find_package(mavros_msgs REQUIRED)
#find_package(mavlink REQUIRED)
#find_package(mavconn REQUIRED)

# Find JSON library (for JSON data handling)
find_package(nlohmann_json REQUIRED)

# Find GStreamer
pkg_check_modules(GST REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
  ${GST_APP_INCLUDE_DIRS}
  ${mavros_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  #${mavlink_INCLUDE_DIRS}
  #${mavconn_INCLUDE_DIRS}

)

# Add executable
add_executable(video src/video.cpp
)

ament_target_dependencies(video
  rclcpp
  cv_bridge
)


add_executable(bluerov_node src/bluerov_node.cpp)
ament_target_dependencies(bluerov_node
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  cv_bridge
  OpenCV
  nlohmann_json
)

#add_executable(bridge_node src/bridge_node.cpp)

# Link libraries
#ament_target_dependencies(bridge_node
  #rclcpp
  #std_msgs
  #mavros
  #mavros_msgs
  #mavlink
#)


# Link libraries
target_link_libraries(video
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

target_link_libraries(bluerov_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${GST_LIBRARIES}
  ${GST_APP_LIBRARIES}
)

# Link libraries for the executable
#target_link_libraries(bridge_node
  #${rclcpp_LIBRARIES}
  #${std_msgs_LIBRARIES}
  #${mavros_LIBRARIES}
  #${mavros_msgs_LIBRARIES}
  #${mavlink_LIBRARIES}
  # This should link MAVLink headers if custom path or precompiled library exists
 # mavros::mavconn
#)

# Install target
install(TARGETS
  video
  bluerov_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)




ament_package()
