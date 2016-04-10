cmake_minimum_required(VERSION 2.8.3)
project(uranus_dp)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  message_generation
  geometry_msgs
  joystick
  eigen_conversions
)

find_package(Eigen REQUIRED)
find_package(GTest REQUIRED)

add_message_files(DIRECTORY msg FILES
  ThrusterForces.msg
  State.msg
)

add_service_files(DIRECTORY srv FILES
  SetControlMode.srv
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
  ${GTEST_INCLUDE_DIRS}
)


# Model server
add_library(model_server
  src/model_server.cpp)



# Control allocation node:
add_executable(allocator
  src/allocator/allocator.cpp
  src/allocator/lagrange_allocator.cpp)
target_link_libraries(allocator ${catkin_LIBRARIES})
add_dependencies(allocator uranus_dp_generate_messages_cpp)

# Controller node:
add_executable(controller
  src/controller/controller.cpp
  src/controller/quaternion_pd_controller.cpp
  src/controller/open_loop_controller.cpp
)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller uranus_dp_generate_messages_cpp)

# Setpoint processing node:
add_executable(setpoint_processing src/setpoint_processing.cpp)
target_link_libraries(setpoint_processing ${catkin_LIBRARIES})
add_dependencies(setpoint_processing uranus_dp_generate_messages_cpp)

# State estimator node:
add_executable(state_estimator
  src/state_estimator/state_estimator.cpp
  src/state_estimator/extended_kalman_filter.cpp
)
target_link_libraries(state_estimator ${catkin_LIBRARIES} model_server)
add_dependencies(state_estimator uranus_dp_generate_messages_cpp)
add_dependencies(state_estimator ${joystick_EXPORTED_TARGETS} ${ros_arduino_EXPORTED_TARGETS})

# Open loop mode, used only for testing:
# add_executable(open_loop
#   src/open_loop/open_loop.cpp)
# target_link_libraries(open_loop ${catkin_LIBRARIES})
# add_dependencies(open_loop uranus_dp_generate_messages_cpp)
# add_dependencies(open_loop ${joystick_EXPORTED_TARGETS})

# Test file stuff
catkin_add_gtest(allocator_test src/test/allocator_test.cpp)
target_link_libraries(allocator_test ${catkin_LIBRARIES})

catkin_add_gtest(controller_test src/test/controller_test.cpp)
target_link_libraries(controller_test ${catkin_LIBRARIES})
