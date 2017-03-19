cmake_minimum_required(VERSION 2.8.3)
project(vortex_controller)

# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  vortex
  vortex_msgs
  eigen_conversions
  dynamic_reconfigure
  cmake_modules # Makes Eigen available to find_package()
)

find_package(Eigen3 REQUIRED)
# find_package(GTest  REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
  cfg/Controller.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES vortex_controller
  CATKIN_DEPENDS # Should this list always match that of find package?
  roscpp
  std_msgs
  nav_msgs
  vortex
  vortex_msgs
  eigen_conversions
  dynamic_reconfigure
  cmake_modules
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  # ${GTEST_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(${PROJECT_NAME}_node src/controller_node.cpp
                                    src/controller_ros.cpp
                                    src/quaternion_pd_controller.cpp # necessary?
                                    src/setpoints.cpp
                                    src/state.cpp
)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
