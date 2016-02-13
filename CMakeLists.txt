cmake_minimum_required(VERSION 2.8.3)
project(uranus_dp)

find_package(
  catkin
  REQUIRED
  cmake_modules
  COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(
  Eigen
  REQUIRED
)

add_message_files(
  FILES
  JoystickUranus.msg
  ThrusterForces.msg
  VelocitySetpoints.msg
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${EIGEN_INCLUDE_DIRS}
)
add_definitions(
  ${EIGEN_DEFINITIONS}
)

add_executable(placeholder src/placeholder.cpp)
target_link_libraries(placeholder ${catkin_LIBRARIES})
# add_dependencies(placeholder uranus_dp_generate_messages_cpp)

add_executable(allocation src/allocation.cpp)
target_link_libraries(allocation ${catkin_LIBRARIES})
# add_dependencies(allocation uranus_dp_generate_messages_cpp)

add_message_files(
  DIRECTORY msg
  FILES JoystickUranus.msg ThrusterForces.msg
)

generate_messages()

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  message_runtime
  DEPENDS
  Eigen
)
