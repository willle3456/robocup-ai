#!/bin/bash
source devel/setup.bash
for t in $(find src/basic_skills -name \*.test); do
    tmp=$(echo $t | awk -F / '{print $5}')
    #echo $tmp
    rostest basic_skills $tmp
done
