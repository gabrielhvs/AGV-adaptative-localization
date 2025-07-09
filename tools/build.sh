#!/bin/bash
docker build -t pipeline-ubuntu2004-ros-noetic-gazebo-$1 . -f $1/Dockerfile --build-arg DIR_NAME=$1

