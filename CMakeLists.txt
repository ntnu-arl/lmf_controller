cmake_minimum_required(VERSION 2.8.3)
project(lmf_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  sensor_msgs
  tf
  cmake_modules
)

find_package(mav_msgs REQUIRED  
  PATHS /opt/ros/melodic/share
  NO_DEFAULT_PATH
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES pid
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs tf
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(pid
  src/library/pid.cpp
)

target_link_libraries(pid ${catkin_LIBRARIES})
add_dependencies(pid ${catkin_EXPORTED_TARGETS})

add_executable(lmf_control_node src/nodes/lmf_controller.cpp)
add_dependencies(lmf_control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(lmf_control_node pid ${catkin_LIBRARIES})

install(TARGETS lmf_control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
