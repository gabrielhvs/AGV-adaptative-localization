#!/bin/bash
docker build -t ghvs/pipeline-ros-noetic-ubuntu-20-02 . -f  $1/Dockerfile --progress=plain