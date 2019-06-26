#!/bin/bash

set -m
source devel/setup.bash
$HOME/Documents/grSim/bin/grsim &
P1=$!
sleep 2
rostest sensor_processing test_all.test &&
fg
kill $P1
