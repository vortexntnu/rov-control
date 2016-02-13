cmake_minimum_required(VERSION 2.8.3)
project(uranus_dp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)

add_message_files(
    FILES
    Joystick.msg
    ThrusterForces.msg
    VelocitySetpoints.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${EIGEN_INCLUDE_DIRS}
)

add_definitions(${EIGEN_DEFINITIONS})

add_message_files(DIRECTORY msg FILES Joystick.msg ThrusterForces.msg)
generate_messages( DEPENDENCIES )

catkin_package(
    CATKIN_DEPENDS std_msgs message_runtime
    DEPENDS Eigen
)

add_executable(allocation src/allocation.cpp)
target_link_libraries(allocation ${catkin_LIBRARIES})

add_executable(placeholder src/placeholder.cpp)
target_link_libraries(placeholder ${catkin_LIBRARIES})



