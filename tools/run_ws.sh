#!/bin/bash

DIR="src/_deps"

if [ ! -d "$DIR" ]; then
  mkdir -p "$DIR"
  vcs import $DIR < dependencies.repos
fi

vcs pull $DIR

catkin build