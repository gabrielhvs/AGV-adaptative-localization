cmake_minimum_required(VERSION 3.0.2)
project(gen_virtual_line)

# use C++17
add_compile_options(-std=c++17)

## Find catkin libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  )

  ###################################
  ## catkin specific configuration ##
  ###################################
  catkin_package(
    CATKIN_DEPENDS
      roscpp
      sensor_msgs
      geometry_msgs
  )

  ###########
  ## Build ##
  ###########

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

catkin_install_python(PROGRAMS
  src/filter_bag.py
  src/gen_virtual_line.py
  src/publishing_waypoints.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
