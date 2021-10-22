cmake_minimum_required(VERSION 2.8.3)
project(lingao_follower)

add_definitions(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED 
COMPONENTS 
cv_bridge 
roscpp 
rospy 
std_msgs 
visualization_msgs
depth_image_proc
dynamic_reconfigure
image_transport
)

find_package(OpenCV REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
   cfg/HSV_reconfigure.cfg
   cfg/Follower.cfg
)

catkin_package(
  INCLUDE_DIRS  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  visualization_msgs 
  depth_image_proc 
  dynamic_reconfigure
)

## Executables
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_library(lingao_line_follower 
    src/linedetect.cpp
)

# hsv_detection node
add_executable(hsv_detection_node src/hsv_detection.cpp)
target_link_libraries(hsv_detection_node lingao_line_follower ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(hsv_detection_node ${catkin_EXPORTED_TARGETS})

# lingao_line_follow node
add_executable(lingao_line_follow_node src/lingao_line_follow.cpp)
target_link_libraries(lingao_line_follow_node lingao_line_follower ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(lingao_line_follow_node ${catkin_EXPORTED_TARGETS})

# lingao_depth_follow node
add_executable(lingao_depth_follow_node src/lingao_depth_follower.cpp)
target_link_libraries(lingao_depth_follow_node  ${catkin_LIBRARIES})
add_dependencies(lingao_depth_follow_node  ${catkin_EXPORTED_TARGETS})
