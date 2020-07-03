#!/bin/bash
# Export CI environment info: https://github.com/mikeferguson/code_coverage

echo "Total Arguments:" $#
echo "All Arguments values:" $@

cd $1

export PROJECT_NAME="cnr_logger"
export CODECOV_TOKEN="dc6c3b82-47d4-4d2d-99a4-0aa76a4640ef"
echo "Generating coverage for $PROJECT_NAME"

export PROJECT_NAME_COVER_REPORT= $PACKAGE_NAME"_coverage_report"

catkin_make -DENABLE_COVERAGE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug $PROJECT_NAME_COVER_REPORT

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
if [ -f "build/$PROJECT_NAME/"$PROJECT_NAME"_coverage_report.info.cleaned" ]; then
    rm "build/$PROJECT_NAME/"$PROJECT_NAME"_coverage_report.info.cleaned"
fi
if [ -f "build/$PROJECT_NAME/"$PROJECT_NAME"_coverage_report.info.removed" ]; then
    rm "build/$PROJECT_NAME/"$PROJECT_NAME"_coverage_report.info.removed"
fi 

echo "==================================================================="
CODECOV_WORKING_DIR="build/$PROJECT_NAME/"$PROJECT_NAME"_coverage_report/codecov"
mkdir -p $CODECOV_WORKING_DIR
cd $CODECOV_WORKING_DIR
source "$1/src/$PROJECT_NAME/.codecov" -s "$1/build/$PROJECT_NAME/" -t $CODECOV_TOKEN -X coveragepy
cd $1
echo "==================================================================="
