cmake_minimum_required(VERSION 3.0.2)
project(trajectory_optimizer_sample)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  trajectory_optimizer
  roslib
  )

find_package(choreonoid REQUIRED)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_executable(test1
  src/test1.cpp
  )

target_link_libraries(test1
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  )

install(TARGETS test1
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
