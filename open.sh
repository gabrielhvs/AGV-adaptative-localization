#!/bin/bash
XAUTH=/tmp/.$USER.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        touch $XAUTH
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

sufix=$1
container=$2
dir=$PWD/$container/

echo  $sufix
echo  $dir

safe_container_creation() {
    if docker container list | grep -i ubuntu2004-ros-noetic-container-$container; then
        echo Container already exists
    else
        docker run -d -i --rm \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged \
            --network="host" \
            --volume="$XAUTH:$XAUTH:rw" \
            --volume="$dir:/home/user/$container/" \
            --volume="/home/$USER/.ssh/:/home/user/.ssh/" \
	        --volume="/dev:/dev" \
            --name ubuntu2004-ros-noetic-container-$container \
            pipeline-ubuntu2004-ros-noetic-gazebo-$sufix /bin/bash
    fi
}

safe_container_creation

docker exec \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="XAUTHORITY=$XAUTH" \
--workdir /home/user/$container \
-it ubuntu2004-ros-noetic-container-$container /bin/bash