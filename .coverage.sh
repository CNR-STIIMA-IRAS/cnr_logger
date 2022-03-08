#!/bin/bash

echo "Generating coverage for 'cnr_logger'"

ws=~/target_ws
cd $ws

catkin build cnr_logger --no-deps --catkin-make-args run_tests 
catkin build cnr_logger --no-deps --catkin-make-args coverage_report --cmake-args -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug -DUSE_ROS=ON -DENABLE_TESTING=ON

# Remove duplicated information
rm "$ws/build/cnr_logger/coverage_report.info.cleaned"
rm "$ws/build/cnr_logger/coverage_report.info.removed"

