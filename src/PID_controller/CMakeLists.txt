cmake_minimum_required(VERSION 3.0.2)
project(PID_controller)

find_package(catkin REQUIRED COMPONENTS rospy)

## Descomente esta linha:
catkin_python_setup()

catkin_package()