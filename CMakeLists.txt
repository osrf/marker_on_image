cmake_minimum_required(VERSION 3.1.3)
project(marker_on_image)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  eigen_conversions
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  tf
  tf_conversions
  vision_msgs
)

catkin_package(
  CATKIN_DEPENDS
  eigen_conversions
  image_geometry
  image_transport
  roscpp
  sensor_msgs
  tf
  tf_conversions
  vision_msgs
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/marker_on_image.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}_node
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

