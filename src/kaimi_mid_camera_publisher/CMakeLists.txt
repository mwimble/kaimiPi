cmake_minimum_required(VERSION 2.8.3)
project(kaimi_mid_camera_publisher)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  rospy
  std_msgs
)

find_package(raspicam REQUIRED cv)
find_package(OpenCV REQUIRED core)
MESSAGE( STATUS "OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS} )
MESSAGE( STATUS "OpenCV_LIBRARIES: " ${OpenCV_LIBRARIES} )
MESSAGE( STATUS "OpenCV_LIBRARY_DIRS: " ${OpenCV_LIBRARY_DIRS} )
MESSAGE( STATUS "OpenCV_VERSION: " ${OpenCV_VERSION} )
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kaimi_mid_camera_publisher
#  CATKIN_DEPENDS cv_bridge dynamic_reconfigure image_transport roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	include
	cfg/cpp
  	${catkin_INCLUDE_DIRS}
  	${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kaimi_mid_camera ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(
	kaimi_mid_camera_publisher_node
	src/FindObject.cpp
	src/kaimi_mid_camera_publisher_node.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(
	kaimi_mid_camera_publisher_node
 	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(kaimi_mid_camera_publisher_node
#  	opencv_ocl
  	raspicam_cv
  	/opt/vc/lib/libvcos.so
  	${OpenCV_CV_LIBS}
	${catkin_LIBRARIES}
	${raspicam_CV_LIBS} 
	/usr/lib/arm-linux-gnueabihf/libopencv_ocl.so
)

