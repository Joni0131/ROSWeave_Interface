#!/bin/bash

#set -x

# this script is for convenience, and to avoid mistakes.

# default values
force_rebuild=false
image_name="ros_weave"
tag_name="latest"
p_out=9999
transport_layer="udp4"
local_workspace="./nav2_ws"
temp_path="/tmp"
dockerfile_path="./mycontainer"

# parse command line arguments
while getopts ":h:f:i:t:p:l:v:d:b:" opt; do
  case $opt in
    h)
    echo "Usage: $0 [-h] [-p path_to_output]"
    echo "  -h: Show this help message"
    echo "  -f: force rebuild of the image"
    echo "  -i: image name (default: ROSweave)"
    echo "  -t: tag name (default: latest)"
    echo "  -p: port for micro-ROS agent (default: 9999)"
    echo "  -l: transport layer protocol (default: udp4)"
    echo "  -v: local workspace path (default: ./nav2_ws)"
    echo "  -d: path to default necessary temp files (default: /tmp)"
    echo "  -b: path to dockerfile (default: ./mycontainer"
    exit 0
    ;;
    f) force_rebuild=true
    ;;
    i) image_name="$OPTARG"
    ;;
    t) tag_name="$OPTARG"
    ;;
    p) p_out="$OPTARG"
    ;;
    l) transport_layer="$OPTARG"
    ;;
    v) local_workspace="$OPTARG"
    ;;
    d) temp_path="$OPTARG"
    ;;
    b) dockerfile_path="$OPTARG"
    ;;
    \?) echo "Invalid option -$OPTARG" >&2
    exit 1
    ;;
  esac
done

# stop all running containers using the specified image
running_containers=$(podman ps -q --filter ancestor="$image_name:$tag_name")
if [ -n "$running_containers" ]; then
    echo "Stopping running containers using image $image_name:$tag_name..."
    podman stop $running_containers
fi

# check if the tagged image already exists
image_exists=$(podman images -q "$image_name:$tag_name")

echo "force_rebuild: $force_rebuild"

# if the image exists and force_rebuild is true, remove the existing image
if [ "$force_rebuild" = true ] && [ -n "$image_exists" ]; then
    echo "Force rebuild is set or image $image_name:$tag_name already exists."
    echo "Removing existing image..."
    podman rmi "$image_name:$tag_name"
fi

# possible build the image
if [ -z "$image_exists" ] || [ "$force_rebuild" = true ]; then
    echo "Building image $image_name:$tag_name..."
    podman build -t "$image_name:$tag_name" -f "$dockerfile_path/Dockerfile" "$dockerfile_path"
fi

# check if the the gazebo temp path exists, if not create it
if [ ! -d "$temp_path/.gazebo" ]; then
    echo "Creating directory $temp_path/.gazebo"
    mkdir -p "$temp_path/.gazebo"
fi

# check if the the X11 temp path exists, if not create it
if [ ! -d "$temp_path/.X11-unix" ]; then
    echo "Creating directory $temp_path/.X11-unix"
    mkdir -p "$temp_path/.X11-unix"
fi

# run the container with the specified parameters and save the container name
container_name=$(podman run -it -d --rm --net=host --privileged \
    -v "$local_workspace":/root/nav2_ws \
    -v "${XAUTHORITY}:/root/.Xauthority" \
    --env="DISPLAY=$DISPLAY" \
    -v "$temp_path/.gazebo/:/root/.gazebo/" \
    -v "$temp_path/.X11-unix:/tmp/.X11-unix:rw" \
    --shm-size=1000mb \
    "localhost/$image_name:$tag_name" \
    "${transport_layer}" -p $p_out)

echo "Container started with name: $container_name"

# in the same container start the nav2 example
mystartupcommand="source /opt/ros/jazzy/setup.bash && source /root/uros_ws/install/setup.bash && \
    python3 -m venv /root/nav2_ws/venv && \
    source /root/nav2_ws/venv/bin/activate && \
    pip install -r /root/nav2_ws/requirements.txt && \
    python3 /root/nav2_ws/launch/preview_launch.py"

podman exec -it "$container_name" bash -c "$mystartupcommand"


