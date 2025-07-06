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

safe_container_creation() {
    if docker container list | grep -i pipeline-ros-noetic-ubuntu-20-02-ws; then
        echo Container already exists
    else
        docker run -d -i --rm \
            -v /tmp/.X11-unix:/tmp/.X11-unix:rw --privileged \
            --network="host" \
            --volume="$XAUTH:$XAUTH:rw" \
            --volume="$PWD:/home/user/ws/" \
            --volume="/home/$USER/.ssh/:/home/user/.ssh/" \
            --name pipeline-ros-noetic-ubuntu-20-02-ws \
            ghvs/pipeline-ros-noetic-ubuntu-20-02 /bin/bash
    fi
}

safe_container_creation

docker exec \
--env="DISPLAY=$DISPLAY" \
--env="QT_X11_NO_MITSHM=1" \
--env="XAUTHORITY=$XAUTH" \
--workdir /home/user/ws \
-it pipeline-ros-noetic-ubuntu-20-02-ws /bin/bash