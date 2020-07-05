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

catkin build -v --no-deps "$PROJECT_NAME" --catkin-make-args "$PROJECT_NAME_COVER_REPORT"

echo "Uploading coverage results to codecov.io"

# Remove duplicated information
ls "$1/build/$PROJECT_NAME/$PROJECT_NAME_COVER_REPORT.*"
if [ -f "$1/build/$PROJECT_NAME/$PROJECT_NAME_COVER_REPORT.info.cleaned" ]; then
    rm "$1/build/$PROJECT_NAME/$PROJECT_NAME_COVER_REPORT.info.cleaned"
fi
if [ -f "$1/build/$PROJECT_NAME/$PROJECT_NAME_COVER_REPORT.info.removed" ]; then
    rm "$1/build/$PROJECT_NAME/$PROJECT_NAME_COVER_REPORT.info.removed"
fi

echo "==================================================================="
CODECOV_WORKING_DIR="build/$PROJECT_NAME/$PROJECT_NAME_COVER_REPORT/codecov"
mkdir -p "$CODECOV_WORKING_DIR"
cd "$CODECOV_WORKING_DIR"
bash <(curl -s https://codecov.io/bash) -t "$CODECOV_TOKEN" -s "$1/build/$PROJECT_NAME/" -X gcov -X coveragepy -X fix -q n.txt -Z -B master -C bb946ec4528b0bc83f76427bc124ef25447d014e
"Result $?"
cd "$CWD"
echo "==================================================================="
