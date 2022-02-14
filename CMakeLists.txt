cmake_minimum_required(VERSION 2.8.3)
project(spark_csv_to_bag)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geodesy
  std_msgs
  rosbag
  sensor_msgs
  cv_bridge
  message_generation
  image_transport  
  image_geometry
)

find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})





###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES spark_csv_to_bag
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



add_executable(spark_csv_to_bag src/flightdata_to_bag.cpp)
target_link_libraries(spark_csv_to_bag ${catkin_LIBRARIES} )

add_executable(overlay_on_image_stream src/overlay_on_image_stream.cpp)
target_link_libraries(overlay_on_image_stream ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

 
add_executable(images_to_bag src/images_to_bag.cpp)
target_link_libraries(images_to_bag ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
 
