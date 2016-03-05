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
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(open_loop_controller src/open_loop_controller.cpp)
target_link_libraries(open_loop_controller ${catkin_LIBRARIES})
add_dependencies(open_loop_controller uranus_dp_generate_messages_cpp)
# Not sure if the next line is needed
add_dependencies(open_loop_controller ${joystick_EXPORTED_TARGETS})

add_executable(allocation src/allocation.cpp)
target_link_libraries(allocation ${catkin_LIBRARIES})
add_dependencies(allocation uranus_dp_generate_messages_cpp)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller uranus_dp_generate_messages_cpp)

add_executable(setpoint_processing src/setpoint_processing.cpp)
target_link_libraries(setpoint_processing ${catkin_LIBRARIES})
add_dependencies(setpoint_processing uranus_dp_generate_messages_cpp)
