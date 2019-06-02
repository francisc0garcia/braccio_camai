cmake_minimum_required(VERSION 2.8.3)
project(braccio_camai)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

catkin_package(
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(MoveitJointInterface src/MoveitJointInterface.cpp)
target_link_libraries(MoveitJointInterface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(DirectJointInterface src/DirectJointInterface.cpp)
target_link_libraries(DirectJointInterface ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#############
## Install ##
#############

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(
  PROGRAMS src/face_detection_edge_tpu.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS 
  MoveitJointInterface 
  DirectJointInterface
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############
## Testing ##
#############

