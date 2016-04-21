cmake_minimum_required(VERSION 2.8.3)
project(rosberrypi_cam)

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/local/lib/cmake/)
#SET(FLAGS "-g -ggdb -g3 -Wl,-rpath-link,${PIROOT}/opt/vc/lib -Wl,-rpath-link,${PIROOT}/lib/arm-linux-gnueabihf -Wl,-rpath-link,${PIROOT}/usr/lib/arm-linux-gnueabihf -Wl,-rpath-link,${PIROOT}/usr/local/lib")

SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -std=gnu++0x" )

find_package(catkin REQUIRED COMPONENTS
	camera_info_manager
	cv_bridge
	image_transport
	roscpp
)

find_package(raspicam REQUIRED cv)
find_package(OpenCV REQUIRED core ocl)

catkin_package()

IF (OpenCV_FOUND AND raspicam_CV_FOUND)
  MESSAGE(STATUS "COMPILING rosberrypi_cam")
  add_executable(
    rosberrypi_cam_node
    src/main.cpp
  )

  target_link_libraries(
  	rosberrypi_cam_node
  	raspicam_cv
    ${raspicam_CV_LIBS} 
  	opencv_video
      	/opt/vc/lib/libmmal_core.so
    vcos
    ${catkin_LIBRARIES}
  )
ELSE()
  MESSAGE(FATAL_ERROR "OPENCV NOT FOUND IN YOUR SYSTEM")
ENDIF()

install(TARGETS
  rosberrypi_cam_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
