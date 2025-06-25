cmake_minimum_required(VERSION 3.0.2)
project(adaptative_localization)

# use C++11
add_compile_options(-std=c++17)


## Find catkin libraries
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
    INCLUDE_DIRS
      include
    CATKIN_DEPENDS
      roscpp
      sensor_msgs
      geometry_msgs
      gazebo_msgs
  )
  
  ###########
  ## Build ##
  ###########
  
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
  )
  
  # Declare a C++ executable
  add_executable(${PROJECT_NAME}
    src/generateLine.cpp
  )
  
  # Specify libraries to link a library or executable target against
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${GAZEBO_LIBRARIES}
  )
  
  #############
  ## Install ##
  #############
  
  # Mark executables for installation
  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )


catkin_install_python(PROGRAMS
  scripts/filter_bag.py
  scripts/gen_virtual_line.py
  scripts/play_process.py
  scripts/simplify_plot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

