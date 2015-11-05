#!/bin/bash
while true; do 
    roslaunch experiment_console structure_frames.launch &
    sleep 10
    stdbuf -oL rostopic hz /camera/depth/camera_info > cameraLog &
    sleep 2
    IS_DEAD=`tail -n 1 cameraLog | grep "no new messages" | wc -l`
    echo "Camera node is alive? $IS_DEAD"
    while [ $IS_DEAD -ne 1 ]; do
	IS_DEAD=`tail -n 1 cameraLog | grep "no new messages" | wc -l`
	sleep 2
    done
    rm cameraLog
    echo "Camera node is not responding, restarting it"
    JOBS=`jobs -p`
    for J in $JOBS 
    do
	pkill -TERM -P $J && kill -9 $J
    done
    echo "Killed camera nodes, restarting in 10 seconds"
    sleep 3 
done
