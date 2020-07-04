#!/bin/bash
# Export CI environment info: https://github.com/mikeferguson/code_coverage

echo "Total Arguments:" $#
echo "All Arguments values:" $@
CWD=$(pwd)
echo "$CWD"
cd "$1"

PROJECT_NAME=$2
CODECOV_TOKEN=$3
PROJECT_NAME_COVER_REPORT=$PROJECT_NAME"_coverage_report"

echo "Generating coverage for $PROJECT_NAME"
echo "Cover report target $PROJECT_NAME_COVER_REPORT"
echo "Codecov Token $CODECOV_TOKEN"

catkin build -v --no-deps "$PACKAGE_NAME" --catkin-make-args "$PROJECT_NAME_COVER_REPORT"

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
if [ -f "build/$PROJECT_NAME/$PROJECT_NAME_coverage_report.info.cleaned" ]; then
    rm "build/$PROJECT_NAME/$PROJECT_NAME_coverage_report.info.cleaned"
fi
if [ -f "build/$PROJECT_NAME/$PROJECT_NAME_coverage_report.info.removed" ]; then
    rm "build/$PROJECT_NAME/$PROJECT_NAME_coverage_report.info.removed"
fi 

echo "==================================================================="
CODECOV_WORKING_DIR="build/$PROJECT_NAME/$PROJECT_NAME_coverage_report/codecov"
mkdir -p $CODECOV_WORKING_DIR
cd $CODECOV_WORKING_DIR
"$1/src/$PROJECT_NAME/./.codecov" -s "$1/build/$PROJECT_NAME/" -t "$CODECOV_TOKEN" -X coveragepy
echo "Result $?"
cd "$CWD"
echo "==================================================================="
