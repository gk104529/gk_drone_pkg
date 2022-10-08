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
    'roslaunch gk_drone_pkg yolo_to_tf.launch'

# Second: Gain tuner
$BYOBU select-pane -t 1
run_command 1 \
    'roslaunch gk_drone_pkg rosbag_recorder.launch'

$BYOBU select-pane -t 2
run_command 2 \
    'roslaunch dji_osdk_ros dji_vehicle_node.launch'

# --------------------------------------------------------------------------


$BYOBU select-window -t main

$BYOBU split-window -h
$BYOBU select-pane -t 0
run_command 0 \
    'docker run --rm -it   --env DISPLAY=$DISPLAY   --device=/dev/dri:/dev/dri   --device /dev/snd:/dev/snd  --volume "/dev:/dev"  --volume "/home/dji/docker_ros:/workspace/docker_ros"   --ipc host   --memory=20096m   --network=host   --cap-add=ALL   --privileged  gkfix:yolov7ros' \
    'sleep 4'\
    'roslaunch yolov7_ros yolov7.launch'



$BYOBU attach-session -t $SESSION
