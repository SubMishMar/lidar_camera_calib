cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 14)
project(random_frame_generator)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  roscpp
  sensor_msgs
  image_transport
)

find_package(Boost 1.45.0 COMPONENTS filesystem regex)

find_package( PCL REQUIRED)
find_package( OpenCV REQUIRED )
catkin_package(
 INCLUDE_DIRS
 CATKIN_DEPENDS
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
  DEPENDS PCL
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/random_frame_generator_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${Boost_LIBRARIES}
)
