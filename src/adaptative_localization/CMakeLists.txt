cmake_minimum_required(VERSION 3.0.2)
project(adaptative_localization)

## Use C++17
add_compile_options(-std=c++17)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  gazebo_msgs
)

find_package(gazebo REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs gazebo_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

## Executável: generate_world_online
add_executable(generate_world_online src/generate_world_online.cpp)
target_link_libraries(generate_world_online
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

## Executável: generate_world_offline
add_executable(generate_world_offline src/generate_world_offline.cpp)
target_link_libraries(generate_world_offline
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS generate_world_online generate_world_offline
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
