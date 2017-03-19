cmake_minimum_required(VERSION 2.8.3)
project(vortex_allocator)

# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  geometry_msgs
  eigen_conversions
  vortex_msgs
  vortex # not sure if needed
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vortex_allocator
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  eigen_conversions
  vortex_msgs
  vortex # need this?
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS} # maybe needed
)

add_executable(${PROJECT_NAME}_node src/allocator_node.cpp
                                    src/allocator_ros.cpp
                                    src/pseudoinverse_allocator.cpp # need this?
)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
