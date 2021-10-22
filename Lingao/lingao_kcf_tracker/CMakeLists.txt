cmake_minimum_required(VERSION 2.8.3)
project(lingao_kcf_tracker)

##################
## Dependencies ##
##################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
  message( "\n${PROJECT_NAME}...")
  message( STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
  message( STATUS " - Libraries: ${OpenCV_LIBRARIES}")
else()
  message( STATUS "OpenCV is required for this sample to do the target tracking")
endif ()

unset(KCFcpp_DIR)
find_path(KCFcpp_DIR
        NAMES
        KCFCpp.sh
        PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/KCFcpp
        )

if(NOT KCFcpp_DIR)
  message(STATUS "Target Tracking sample depends on KCFcpp library from https://github.com/joaofaro/KCFcpp.git")
  message(STATUS "When you run make, we will try to clone the repo")
  message(STATUS "This will happen only once as long as you do not delete it")

  # include(${CMAKE_MODULE_PATH}/DownloadProject/DownloadProject.cmake)
  # download_project(PROJ                kcf_cpp
  #         GIT_REPOSITORY      https://github.com/joaofaro/KCFcpp.git
  #         GIT_TAG             master
  #         ${UPDATE_DISCONNECTED_IF_AVAILABLE}
  #         SOURCE_DIR          ${CMAKE_CURRENT_SOURCE_DIR}/KCFcpp
  #         )

endif()

#依赖包
find_package(catkin REQUIRED COMPONENTS
roscpp 
cv_bridge
std_msgs
sensor_msgs
image_transport
geometry_msgs
)

 
################################################
## Declare ROS messages, services and actions ##
################################################



###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
)

include_directories(
  include
  KCFcpp/src
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


###########
## Build ##
###########

## Declare a C++ library
add_library(KCFcpp STATIC
  KCFcpp/src/fhog.cpp
  KCFcpp/src/kcftracker.cpp)

add_executable(lingao_kcf_tracking_node
  ${SOURCE_FILES}
  src/target_tracking.cpp
  src/tracking_utility.cpp
)

target_link_libraries(lingao_kcf_tracking_node
  KCFcpp
  ${OpenCV_LIBRARIES}
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############


#############
## Testing ##
#############

