#!/bin/bash
#trap 'kill $P1; exit' SIGINT
set -m
source devel/setup.bash
$HOME/Documents/grSim/bin/grsim --headless &
P1=$!
roslaunch robocup_master master.launch &
P2=$!
python $HOME/Documents/robocup-ai/src/robocup_master/src/robocup_master/config_loader.py &&
fg
echo $P1
kill $P1
