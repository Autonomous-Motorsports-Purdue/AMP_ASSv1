if [ "$#" -eq 2 ]; then
    docker build --file ./docker/amd64.Dockerfile --tag amp-assv1:dev .
fi
if [ "$1" -eq 0 ]; then # Intel graphics
    DOCKER_ARGS="-it --rm --net=host \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --device="/dev/dri:/dev/dri" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --privileged"
elif [ "$1" -eq 1 ]; then # Nvidia graphics
    DOCKER_ARGS="-it --rm --net=host \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_DRIVER_CAPABILITIES=all" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --gpus all \
        --privileged"
fi
xhost + local:docker > /dev/null
docker run $DOCKER_ARGS amp-assv1:dev bash
