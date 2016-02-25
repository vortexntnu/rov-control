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
  geometry_msgs
)

find_package(
  Eigen
  REQUIRED
)

add_message_files(
  DIRECTORY
  msg
  FILES
  JoystickUranus.msg
  ThrusterForces.msg
)

generate_messages()


include_directories(
  include ${catkin_INCLUDE_DIRS}
)
include_directories(
  ${EIGEN_INCLUDE_DIRS}
)
add_definitions(
  ${EIGEN_DEFINITIONS}
)

add_executable(placeholder src/placeholder.cpp)
target_link_libraries(placeholder ${catkin_LIBRARIES})
add_dependencies(placeholder uranus_dp_generate_messages_cpp)

add_executable(allocation src/allocation.cpp)
target_link_libraries(allocation ${catkin_LIBRARIES})
add_dependencies(allocation uranus_dp_generate_messages_cpp)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller uranus_dp_generate_messages_cpp)

# add_executable(setpoint_filter src/setpoint_filter.cpp)
# target_link_libraries(setpoint_filter ${catkin_LIBRARIES})
# add_dependencies(allocation uranus_dp_generate_messages_cpp)

