#!/usr/bin/env bash

set -e

IMAGE_NAME=bluerov2_ros2
CONTAINER_NAME=bluerov2_ros2

USE_GPU=false

# -------------------------------------------------
# Parse arguments
# -------------------------------------------------
for arg in "$@"; do
  case "$arg" in
    --gpu)
      USE_GPU=true
      ;;
    *)
      echo "Unknown option: $arg"
      echo "Usage: ./run_docker.sh [--gpu]"
      exit 1
      ;;
  esac
done

# -------------------------------------------------
# Allow X11 access
# -------------------------------------------------
xhost +local:docker

# -------------------------------------------------
# Remove existing container
# -------------------------------------------------
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  docker rm -f "${CONTAINER_NAME}"
fi

# -------------------------------------------------
# Base docker run command
# -------------------------------------------------
DOCKER_CMD="docker run -it \
  --name ${CONTAINER_NAME} \
  --net=host \
  --ipc=host \
  --privileged \
  --env DISPLAY=${DISPLAY} \
  --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --volume /dev:/dev"

# -------------------------------------------------
# Optional GPU support
# -------------------------------------------------
if [ "${USE_GPU}" = true ]; then
  echo "[INFO] Running with GPU support enabled"
  DOCKER_CMD="${DOCKER_CMD} \
    --gpus all \
    --env NVIDIA_VISIBLE_DEVICES=all \
    --env NVIDIA_DRIVER_CAPABILITIES=all"
else
  echo "[INFO] Running without GPU support"
fi

# -------------------------------------------------
# Run container
# -------------------------------------------------
exec ${DOCKER_CMD} ${IMAGE_NAME}
