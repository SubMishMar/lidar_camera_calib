cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 14)
project(calibration_backend_optimization)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  velodyne_pointcloud
  normal_msg
  line_msg
)

find_package( PCL REQUIRED)
find_package( Ceres REQUIRED)
find_package( OpenCV REQUIRED )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(calibration_node_test_initialization_node src/calibration_node_test_initialization.cpp)
target_link_libraries(calibration_node_test_initialization_node
   ${catkin_LIBRARIES}
   ${CERES_LIBRARIES}
   ${OpenCV_LIBS}
)


