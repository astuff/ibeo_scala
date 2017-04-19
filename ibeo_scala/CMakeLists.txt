cmake_minimum_required(VERSION 2.8.3)
project(ibeo_scala)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED
  COMPONENTS
  std_msgs
  ibeo_lux_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ibeo_scala
  CATKIN_DEPENDS ibeo_lux_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(core_ibeo_scala/include)

add_executable(${PROJECT_NAME}
  src/ibeo_scala.cpp
  core_ibeo_scala/src/ibeo_scala_core.cpp
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
