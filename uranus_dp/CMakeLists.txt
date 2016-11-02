cmake_minimum_required(VERSION 2.8.3)
project(uranus_dp)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  cmake_modules
  roscpp
  message_generation
  eigen_conversions
  vortex_msgs
  nav_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

# add_service_files(
#   FILES
#   SetControllerGains.srv
#   ResetEstimator.srv
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/uranusDp.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  cmake_modules
  roscpp
  message_generation
  eigen_conversions
  vortex_msgs
  nav_msgs
  sensor_msgs
  dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${GTEST_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(controller src/controller/controller_node.cpp
                          src/controller/controller.cpp
                          src/controller/quaternion_pd_controller.cpp
)

add_executable(allocator src/allocator/allocator_node.cpp
                         src/allocator/pseudoinverse_allocator.cpp
)

add_executable(estimator src/estimator/estimator_node.cpp
                         src/estimator/integration_filter.cpp
                         src/estimator/simple_estimator.cpp
)

add_dependencies(controller ${PROJECT_NAME}_gencfg)
# add_dependencies(allocator)
# add_dependencies(estimator)

target_link_libraries(controller ${catkin_LIBRARIES})
target_link_libraries(allocator  ${catkin_LIBRARIES})
target_link_libraries(estimator  ${catkin_LIBRARIES})

#############
## Testing ##
#############

# catkin_add_gtest(controller_test              test/controller_test.cpp)
# catkin_add_gtest(depth_hold_test              test/depth_hold_test.cpp)
# catkin_add_gtest(allocator_test               test/allocator_test.cpp)
# catkin_add_gtest(integration_filter_test      test/integration_filter_test.cpp)
# catkin_add_gtest(open_loop_integration_test   test/open_loop_integration_test.cpp)
# catkin_add_gtest(closed_loop_integration_test test/closed_loop_integration_test.cpp)

# target_link_libraries(allocator_test               ${catkin_LIBRARIES})
# target_link_libraries(controller_test              ${catkin_LIBRARIES})
# target_link_libraries(depth_hold_test              ${catkin_LIBRARIES})
# target_link_libraries(integration_filter_test      ${catkin_LIBRARIES})
# target_link_libraries(open_loop_integration_test   ${catkin_LIBRARIES})
# target_link_libraries(closed_loop_integration_test ${catkin_LIBRARIES})
