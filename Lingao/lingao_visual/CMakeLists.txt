cmake_minimum_required(VERSION 2.8.3)
project(lingao_visual)

##################
## Dependencies ##
##################

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
#依赖包
find_package(catkin REQUIRED 
    # tf2
		# tf2_ros
    # geometry_msgs 
    # sensor_msgs 
    # std_msgs
    # dynamic_reconfigure
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
