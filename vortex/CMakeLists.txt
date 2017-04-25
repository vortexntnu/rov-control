cmake_minimum_required(VERSION 2.8.3)
project(vortex)

# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  vortex_msgs
  roslint
)

set(ROSLINT_CPP_OPTS "--filter=-legal/copyright")
roslint_cpp()

catkin_package(
  INCLUDE_DIRS
  include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    ${PROJECT_NAME}-test
    test/integration_test.test
    test/integration_test.cpp
  )

  target_link_libraries(${PROJECT_NAME}-test
    ${catkin_LIBRARIES}
  )

  roslint_add_test()
endif()
