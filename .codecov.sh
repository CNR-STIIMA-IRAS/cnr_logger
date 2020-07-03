#!/bin/bash
# Export CI environment info: https://github.com/mikeferguson/code_coverage

cd ~/catkin_ws

export PROJECT_NAME="cnr_logger"
export CODECOV_TOKEN="dc6c3b82-47d4-4d2d-99a4-0aa76a4640ef"
echo "Generating coverage for $PROJECT_NAME"

export PROJECT_NAME_COVER_REPORT= $PACKAGE_NAME"_coverage_report"

catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug $PROJECT_NAME_COVER_REPORT

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
rm "build/$PROJECT_NAME/robot_calibration_coverage_report.info.cleaned"
rm "build/$PROJECT_NAME/robot_calibration_coverage_report.info.removed"

# Actually upload coverage information
#bash <(curl -s https://codecov.io/bash) -s "build/$PROJECT_NAME/"

curl -s https://codecov.io/bash > .codecov
chmod +x .codecov
./.codecov -s "build/$PROJECT_NAME/" -t $CODECOV_TOKEN -x

