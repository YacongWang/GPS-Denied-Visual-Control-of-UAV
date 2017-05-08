############################
#
# By Zhou Yu (yu.nico.zhou@gmail.com)
# NUS, Singapore 2017.
#
############################
cmake_minimum_required(VERSION 2.8.3)
project(qrcode_detect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  message_filters
  tf_conversions
  tf
  dynamic_reconfigure
  cv_bridge
  image_transport
)

generate_dynamic_reconfigure_options(
        cfg/qrcode_detect.cfg
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

 add_executable(qrcode_detect_node
	src/Marker.cpp
        src/MarkerDetector.cpp
        src/qrcodeDetectMain.cpp
        src/qrcode_detect_node.cpp

	include/Marker.h
        include/MarkerDetector.h
        include/qrcodeDetectMain.h)

 add_dependencies(qrcode_detect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(qrcode_detect_node
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )
