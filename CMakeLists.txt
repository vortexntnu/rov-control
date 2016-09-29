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

find_package(Eigen3 REQUIRED)
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

add_executable(setpoint_processing src/setpoint_processing/setpoint_processing_node.cpp
                                   src/setpoint_processing/setpoint_processing.cpp
)

add_executable(controller src/controller/controller_node.cpp
                          src/controller/controller.cpp
                          src/controller/open_loop_controller.cpp
                          src/controller/depth_hold_controller.cpp
                          src/controller/quaternion_pd_controller.cpp
)

add_executable(allocator src/allocator/allocator.cpp
                         src/allocator/lagrange_allocator.cpp
)

add_executable(integration_filter src/integration_filter/integration_filter_node.cpp
                                  src/integration_filter/integration_filter.cpp
)

add_dependencies(setpoint_processing ${catkin_EXPORTED_TARGETS})
add_dependencies(controller          ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(allocator           ${PROJECT_NAME}_generate_messages_cpp)
add_dependencies(integration_filter  ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(setpoint_processing ${catkin_LIBRARIES})
target_link_libraries(controller          ${catkin_LIBRARIES})
target_link_libraries(allocator           ${catkin_LIBRARIES})
target_link_libraries(integration_filter  ${catkin_LIBRARIES})

#############
## Testing ##
#############

catkin_add_gtest(controller_test              test/controller_test.cpp)
catkin_add_gtest(depth_hold_test              test/depth_hold_test.cpp)
catkin_add_gtest(allocator_test               test/allocator_test.cpp)
catkin_add_gtest(integration_filter_test      test/integration_filter_test.cpp)
catkin_add_gtest(open_loop_integration_test   test/open_loop_integration_test.cpp)
catkin_add_gtest(closed_loop_integration_test test/closed_loop_integration_test.cpp)

target_link_libraries(allocator_test               ${catkin_LIBRARIES})
target_link_libraries(controller_test              ${catkin_LIBRARIES})
target_link_libraries(depth_hold_test              ${catkin_LIBRARIES})
target_link_libraries(integration_filter_test      ${catkin_LIBRARIES})
target_link_libraries(open_loop_integration_test   ${catkin_LIBRARIES})
target_link_libraries(closed_loop_integration_test ${catkin_LIBRARIES})
