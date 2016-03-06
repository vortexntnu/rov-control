cmake_minimum_required(VERSION 2.8.3)
project(uranus_dp)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  joystick
)

find_package(Eigen REQUIRED)

add_message_files(DIRECTORY msg FILES
  ThrusterForces.msg
  State.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS
  cmake_modules
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  joystick
)

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)



# Control allocation node:
add_executable(allocation
  src/allocation.cpp
  src/lagrange_allocator.cpp)
target_link_libraries(allocation ${catkin_LIBRARIES})
add_dependencies(allocation uranus_dp_generate_messages_cpp)

# Controller node:
add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller uranus_dp_generate_messages_cpp)

# Setpoint processing node:
add_executable(setpoint_processing src/setpoint_processing.cpp)
target_link_libraries(setpoint_processing ${catkin_LIBRARIES})
add_dependencies(setpoint_processing uranus_dp_generate_messages_cpp)

# State estimator node:
add_executable(state_estimator src/state_estimator.cpp)
target_link_libraries(state_estimator ${catkin_LIBRARIES})
add_dependencies(state_estimator uranus_dp_generate_messages_cpp)

# Open loop controller, used only for testing:
add_executable(open_loop_controller src/open_loop_controller.cpp)
target_link_libraries(open_loop_controller ${catkin_LIBRARIES})
add_dependencies(open_loop_controller uranus_dp_generate_messages_cpp)
add_dependencies(open_loop_controller ${joystick_EXPORTED_TARGETS})