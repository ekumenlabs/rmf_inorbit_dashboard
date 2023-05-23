#!/bin/bash

set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t run.sh [OPTIONS] \n
  Options:\n
  \t-i --image_name\t\t Name of the image to be run (default inorbit_rmf).\n
  \t-c --container_name\t Name of the container(default inorbit_rmf).\n
  \t--use_nvidia\t\t Use nvidia runtime.\n
  Examples:\n
  \trun.sh\n
  \trun.sh --image_name custom_image_name --container_name custom_container_name \n'
}

# Returns true when the path is relative, false otherwise.
#
# Arguments
#   $1 -> Path
function is_relative_path() {
  case $1 in
    /*) return 1 ;; # false
    *) return 0 ;;  # true
  esac
}

echo "Running the container..."

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -c|--container_name) CONTAINER_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        --use_nvidia) NVIDIA_FLAGS="--gpus=all -e NVIDIA_DRIVER_CAPABILITIES=all" ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

IMAGE_NAME=${IMAGE_NAME:-inorbit_rmf}
CONTAINER_NAME=${CONTAINER_NAME:-inorbit_rmf}

SSH_PATH=/home/$USER/.ssh
WORKSPACE_FOLDER_PATH="$(cd "$(dirname "$0")"; cd ../../..; pwd)"
WORKSPACE_CONTAINER=/home/$(whoami)/ws/
SSH_AUTH_SOCK_USER=$SSH_AUTH_SOCK

# Check if name container is already taken.
if docker container ls -a | grep "${CONTAINER_NAME}$" -c &> /dev/null; then
   printf "Error: Docker container called $CONTAINER_NAME is already opened.     \
   \n\nTry removing the old container by doing: \n\t docker rm $CONTAINER_NAME   \
   \nor just initialize it with a different name.\n"
   exit 1
fi

xhost +
docker run --net=host -it $NVIDIA_FLAGS \
       -e DISPLAY=$DISPLAY \
       -e SSH_AUTH_SOCK=$SSH_AUTH_SOCK_USER \
       -v $(dirname $SSH_AUTH_SOCK_USER):$(dirname $SSH_AUTH_SOCK_USER) \
       -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
       -v $WORKSPACE_FOLDER_PATH/:$WORKSPACE_CONTAINER/ \
       -v $SSH_PATH:$SSH_PATH \
       --name $CONTAINER_NAME $IMAGE_NAME
xhost -

# Trap workspace exits and give the user the choice to save changes.
function onexit() {
  while true; do
    read -p "Do you want to overwrite the image called '$IMAGE_NAME' with the current changes? [y/n]: " answer
    if [[ "${answer:0:1}" =~ y|Y ]]; then
      echo "Overwriting docker image..."
      docker commit $CONTAINER_NAME $IMAGE_NAME
      break
    elif [[ "${answer:0:1}" =~ n|N ]]; then
      break
    fi
  done
  docker stop $CONTAINER_NAME > /dev/null
  docker rm $CONTAINER_NAME > /dev/null
}

trap onexit EXIT
