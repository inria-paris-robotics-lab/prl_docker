#!/bin/bash
set -e

# 1. Check arguments
if [ "$#" -lt 2 ] || [ "$#" -gt 4 ]; then
  echo "Usage: $0 <container_name> <share_directory_path> <image_name(optional)> [--no-gpu] [--rebuild]"
  exit 1
fi

CONTAINER_NAME="$1"
HOST_STORAGE=$(realpath "$2")
IMAGE_TO_USE="prl:mantis-jazzy"
REBUILD=false
USE_GPU=true

# Export them for Docker Compose
export CONTAINER_NAME
export HOST_STORAGE

# Shift out container_name and storage so we can parse remaining args
shift 2

for arg in "$@"; do
  if [ "$arg" = "--no-gpu" ]; then
    USE_GPU=false
  elif [ "$arg" = "--rebuild" ]; then
    REBUILD=true
  else
    IMAGE_TO_USE="$arg"
  fi
done

# Image configuration
IMAGE_NAME="$IMAGE_TO_USE"
export IMAGE_NAME

# Dynamic ID detection for the entrypoint
HOST_UID=$(id -u)
HOST_GID=$(id -g)
HOST_DOCKER_GID=$(getent group docker | cut -d: -f3 || echo 999)
HOST_RENDER_GID=$(getent group render | cut -d: -f3 || echo 107)
HOST_INPUT_GID=$(getent group input | cut -d: -f3 || echo 106)
HOST_VIDEO_GID=$(getent group video | cut -d: -f3 || echo 44)
HOST_DIALOUT_GID=$(getent group dialout | cut -d: -f3 || echo 20)

export HOST_UID
export HOST_GID
export HOST_DOCKER_GID
export HOST_RENDER_GID
export HOST_INPUT_GID
export HOST_VIDEO_GID
export HOST_DIALOUT_GID

# Allow local docker containers to access the X server
xhost +local:docker > /dev/null 2>&1

# Check if GPU support is requested and available
SERVICE_TO_RUN="mantis-cpu"
if [ "$USE_GPU" = true ] && command -v nvidia-smi &> /dev/null; then
    SERVICE_TO_RUN="mantis-gpu"
else
    if [ "$USE_GPU" = true ]; then
        echo "⚠️ NVIDIA drivers not found. Falling back to CPU mode."
    fi
    USE_GPU=false
fi

# --- EXECUTION ---
if [ "$REBUILD" = true ]; then
    echo "🛠 Rebuilding image: $IMAGE_NAME..."
    docker compose -f docker-compose.yml build --no-cache
fi

echo "🚀 Launching $CONTAINER_NAME using $IMAGE_NAME (GPU: $USE_GPU)..."
docker compose run --rm --name "$CONTAINER_NAME" --service-ports "$SERVICE_TO_RUN" bash

# Clean up xhost permissions
xhost -local:docker > /dev/null 2>&1
echo "Container $CONTAINER_NAME exited."
