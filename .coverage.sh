#!/bin/bash

source "${ICI_SRC_PATH}/workspace.sh"
source "${ICI_SRC_PATH}/util.sh"

echo "Generating coverage for $PROJECT_NAME"

catkin build $PROJECT_NAME -v --no-deps --catkin-make-args $PROJECT_NAME_coverage_report

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
rm "$ws/build/robot_calibration/robot_calibration_coverage_report.info.cleaned"
rm "$ws/build/robot_calibration/robot_calibration_coverage_report.info.removed"

# Actually upload coverage information
bash <(curl -s https://codecov.io/bash) -s "$ws/build/robot_calibration/"
