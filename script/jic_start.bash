#!/bin/bash

BYOBU="${BYOBU:-byobu-tmux}"
SESSION='JIC'

run_command() {
    local pane="$1"
    shift

    $BYOBU select-pane -t $pane

    for cmd in "$@"; do
        eval "$BYOBU send-keys '$cmd' Enter"
    done
}


if ! hash $BYOBU 2>/dev/null; then
    echo 'byobu command not found' >&2
    exit 1
fi

if [[ -n $( $BYOBU list-sessions 2>/dev/null | grep $SESSION ) ]]; then
    $BYOBU list-sessions
    echo "Session $SESSION already exists; attaching"
    sleep 2
    $BYOBU attach-session -t $SESSION
    exit 0
fi


$BYOBU -2 new-session -d -s $SESSION
$BYOBU rename-window -t $SESSION:0 'main'

$BYOBU new-window -n docker



# Setting up the panes
# Side-by-side: left for the commands, right for debugging
$BYOBU split-window -h
$BYOBU select-pane -t 0

# Split the left pane into four
$BYOBU split-window -v
$BYOBU select-pane -t 2

$BYOBU split-window -v 
$BYOBU select-pane -t 3

# Running the commands
# --------------------------------------------------------------------------

# First: runs the quince_base launch
run_command 0 \
    'export DISPLAY=":0";roslaunch dji_osdk_ros dji_vehicle_node.launch --wait' 

# Second: Gain tuner
$BYOBU select-pane -t 1
run_command 1 \
    'sleep 2' \
    'roslaunch gk_drone_pkg start_all.launch --wait' 

$BYOBU select-pane -t 2
run_command 2 \
    'sleep 15' \
    'rosservice call /setup_camera_stream "cameraType: 0
start: 1" --wait' 

$BYOBU select-pane -t 3
run_command 3 \
    'sleep 15' \
    'rosservice call /setup_camera_stream "cameraType: 1
start: 1" --wait'


# --------------------------------------------------------------------------


$BYOBU select-window -t main

$BYOBU split-window -h
$BYOBU select-pane -t 0
run_command 0 \
    'docker run --rm -it   --env DISPLAY=$DISPLAY   --device=/dev/dri:/dev/dri   --device /dev/snd:/dev/snd  --volume "/dev:/dev"  --volume "/home/dji/docker_ros:/workspace/docker_ros"   --ipc host   --memory=20096m   --network=host   --cap-add=ALL   --privileged  gkfix:yolov7ros' \
    

$BYOBU attach-session -t $SESSION
