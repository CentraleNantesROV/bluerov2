#!/usr/bin/env bash

set -e

IMAGE_NAME=bluerov2_ros2
DOCKERFILE=Dockerfile

echo "[INFO] Building Docker image: ${IMAGE_NAME}"

docker build \
  -t ${IMAGE_NAME} \
  -f ${DOCKERFILE} \
  .

echo "[INFO] Build completed successfully"
