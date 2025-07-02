#!/bin/bash

DIR="src/_deps"

if [ ! -d "$DIR" ]; then
  mkdir -p "$DIR"
  vcs import $DIR < dependencies.repos
fi

vcs pull $DIR

export GAZEBO_MODEL_PATH=`pwd`/src/_deps/aws_warehouse_world/models

catkin build