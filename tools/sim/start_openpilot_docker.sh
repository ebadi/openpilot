#!/bin/bash

ROS_IP=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | grep 192.168)
ROS_MASTER_URI='http://192.168.1.200:11311'
echo "Openpilot IP address: $ROS_IP \n Gokart IP address: $ROS_MASTER_URI"
BRIDGEENV='gokart'  # options: carla,gokart

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null && pwd)"
cd $DIR

OPENPILOT_DIR="/openpilot"
if ! [[ -z "$MOUNT_OPENPILOT" ]]; then
  OPENPILOT_DIR="$(dirname $(dirname $DIR))"
  EXTRA_ARGS="-v $OPENPILOT_DIR:$OPENPILOT_DIR -e PYTHONPATH=$OPENPILOT_DIR:$PYTHONPATH"
fi

if [[ "$CI" ]]; then
  CMD="CI=1 ${OPENPILOT_DIR}/tools/sim/tests/test_carla_integration.py"
else
  # expose X to the container
  xhost +local:root

  # docker pull ghcr.io/commaai/openpilot-sim:latest
  CMD="./tmux_script.sh $*"
  EXTRA_ARGS="${EXTRA_ARGS} -it -e ROS_MASTER_URI=$ROS_MASTER_URI -e ROS_IP=$ROS_IP -e BRIDGEENV=$BRIDGEENV"
fi



docker kill openpilot_client || true
docker run --net=host\
  --name openpilot_client \
  --rm \
  --gpus all \
  --device=/dev/dri:/dev/dri \
  --device=/dev/input:/dev/input \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$DIR/":/openpilot/tools/sim/ \
  -v "$DIR/hamiddata/":/root/.comma/media/0/realdata/ \
  --device /dev/video0  --device /dev/video1  \
  --shm-size 1G \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -w "$OPENPILOT_DIR/tools/sim" \
  $EXTRA_ARGS \
  master-webcam-openpilot-sim:latest \
  /bin/bash -c "$CMD"
