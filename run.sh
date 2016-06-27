#!/usr/bin/env bash

source devel/setup.bash
roscore > roscore.log &
sleep 5
rosrun tcu tcu 2>&1 > tcu.log &
rosrun navig navig 2>&1 > navig.log &
rosrun compass compass_node -s -o 2>&1 > compass.log &
rosrun motion motion 2>&1 > motion.log &
rosrun mission mission 2>&1 > mission.log &
rosrun supervisor supervisor_node 2>&1 > supervisor.log &
rosrun vision vision_node 2>&1 > vision.log &
rosrun dsp dsp 2>&1 > dsp &
rosrun cmd_id_generator gen_cmd_id.py
