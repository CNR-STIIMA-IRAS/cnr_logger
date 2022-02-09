#!/bin/bash

# source "${ICI_SRC_PATH}/workspace.sh"
# source "${ICI_SRC_PATH}/util.sh"

# echo "Generating coverage for 'cnr_logger'"

ws=~/target_ws
# extend="/opt/ros/$ROS_DISTRO"
# ici_exec_in_workspace "$extend" "$ws"
catkin build cnr_logger -v --no-deps --catkin-make-args coverage_report

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
rm "$ws/build/cnr_logger/coverage_report.info.cleaned"
rm "$ws/build/cnr_logger/coverage_report.info.removed"

# Actually upload coverage information
bash <(curl -s https://codecov.io/bash) -s "$ws/build/cnr_logger/"
