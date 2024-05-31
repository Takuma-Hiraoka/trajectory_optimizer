cmake_minimum_required(VERSION 3.0.2)
project(trajectory_optimizer)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  ik_constraint2
  prioritized_inverse_kinematics_solver2
  )

find_package(choreonoid REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trajectory_optimizer
  CATKIN_DEPENDS prioritized_inverse_kinematics_solver2
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/trajectory_optimizer.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )
