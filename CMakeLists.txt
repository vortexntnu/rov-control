cmake_minimum_required(VERSION 2.8.3)
project(uranus_dp)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  message_generation
  eigen_conversions
  maelstrom_msgs
  nav_msgs
  sensor_msgs
)

find_package(Eigen REQUIRED)
find_package(GTest REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  SetControlMode.srv
  SetControllerGains.srv
  ResetIntegrationFilter.srv
)

generate_messages()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  cmake_modules
  roscpp
  message_generation
  eigen_conversions
  maelstrom_msgs
  nav_msgs
  sensor_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
)

add_executable(setpoint_processing src/setpoint_processing.cpp
)

add_executable(controller src/controller/controller.cpp
                          src/controller/quaternion_pd_controller.cpp
                          src/controller/open_loop_controller.cpp
)

add_executable(allocator src/allocator/allocator.cpp
                         src/allocator/lagrange_allocator.cpp
)

add_executable(state_estimator src/state_estimator/state_estimator.cpp
                               src/state_estimator/integration_filter.cpp
)

add_dependencies(setpoint_processing ${catkin_EXPORTED_TARGETS})
add_dependencies(controller ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(allocator ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(state_estimator ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(setpoint_processing ${catkin_LIBRARIES})
target_link_libraries(controller ${catkin_LIBRARIES})
target_link_libraries(allocator ${catkin_LIBRARIES})
target_link_libraries(state_estimator ${catkin_LIBRARIES})

#############
## Testing ##
#############

catkin_add_gtest(allocator_test test/allocator_test.cpp)
catkin_add_gtest(controller_test test/controller_test.cpp)
catkin_add_gtest(state_estimator_test test/state_estimator_test.cpp)
catkin_add_gtest(open_loop_integration_test test/open_loop_integration_test.cpp)
catkin_add_gtest(closed_loop_integration_test test/closed_loop_integration_test.cpp)

target_link_libraries(allocator_test ${catkin_LIBRARIES})
target_link_libraries(controller_test ${catkin_LIBRARIES})
target_link_libraries(state_estimator_test ${catkin_LIBRARIES})
target_link_libraries(open_loop_integration_test ${catkin_LIBRARIES})
target_link_libraries(closed_loop_integration_test ${catkin_LIBRARIES})