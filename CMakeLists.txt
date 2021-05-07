cmake_minimum_required(VERSION 3.0.2)
project(hikrobot_mv_ros)

# Compile as C++11, supported in ROS Kinetic and newer ==============
add_compile_options(-std=c++11)

# Find catkin macros and libraries ==================================
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  sensor_msgs
  cv_bridge
  image_transport
  camera_info_manager  
)

# find the specific openCV version ==================================
find_package(OpenCV 3.2.0 REQUIRED ) # (problem with version > 4.x.x)

# catkin specific configuration =====================================
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES hikrobot_mv_ros
   CATKIN_DEPENDS 
   nodelet 
   roscpp 
   sensor_msgs
   cv_bridge
   image_transport
   camera_info_manager
)

# Specify additional locations of header files ======================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  /opt/MVS/include/  
)

# the last folder depending of the architecture (64, aarch64, ...)
link_directories(/opt/MVS/lib/aarch64) 
SET(CMAKE_INSTALL_RPATH /opt/MVS/lib/aarch64)

# hikrobot_mv_ros_node ==============================================
add_executable(${PROJECT_NAME}_node 
	src/hikrobot_mv_node.cpp
)

add_dependencies(${PROJECT_NAME}_node 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node  
	${catkin_LIBRARIES}
)
 
# hikrobot_mv_driver ================================================
add_library(hikrobot_mv_driver 
	src/nodelets/hikrobot_mv_driver.cpp
)
	
add_dependencies(hikrobot_mv_driver 
	${${PROJECT_NAME}_EXPORTED_TARGETS} 
	${catkin_EXPORTED_TARGETS}
)
	
target_link_libraries(hikrobot_mv_driver 
	${catkin_LIBRARIES} 
    ${OpenCV_LIBRARIES}	
	MvCameraControl
)

# install hikrobot_mv_ros_node & hikrobot_mv_driver =================
install(TARGETS ${PROJECT_NAME}_node hikrobot_mv_driver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install (nodelet) =================================================
install(FILES 
	nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
