#!/bin/bash
# Export CI environment info

cd ~/catkin_ws

echo "Generating coverage for $PROJECT_NAME"

export PROJECT_NAME_COVER_REPORT= $PACKAGE_NAME"_coverage_report"

catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug $PROJECT_NAME_COVER_REPORT

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
rm "build/$PROJECT_NAME/robot_calibration_coverage_report.info.cleaned"
rm "build/$PROJECT_NAME/robot_calibration_coverage_report.info.removed"

# Actually upload coverage information
bash <(curl -s https://codecov.io/bash) -s "$ws/build/$PROJECT_NAME/"

#curl -s https://codecov.io/bash > .codecov
#chmod +x .codecov
#./.codecov


#ci_env=`bash <(curl -s https://codecov.io/env)`
#export DOCKER_RUN_OPTS="$DOCKER_RUN_OPTS $ci_env"
