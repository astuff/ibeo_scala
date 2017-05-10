cmake_minimum_required(VERSION 2.8.3)
project(ibeo_scala)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11) # Needed to support <tuple>

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  pcl_ros
  network_interface
  ibeo_scala_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  ibeo_scala_msgs
)

include_directories(include/${PROJECT_NAME}
  core_ibeo_scala/include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
  core_ibeo_scala/src/ibeo_scala_core.cpp
  src/ros_msg_converters.cpp
  src/ibeo_scala.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
