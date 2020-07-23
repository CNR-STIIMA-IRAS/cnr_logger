cmake_minimum_required(VERSION 3.1)

project(cnr_logger)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS roscpp roslint)
find_package(Boost REQUIRED COMPONENTS date_time )

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

include_directories( include ${catkin_INCLUDE_DIRS} )

## Add gtest based cpp test target and link libraries
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_logger.cpp )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )
roslint_cpp( src/cnr_logger.cpp )

add_executable( ${PROJECT_NAME}_example src/cnr_logger_example.cpp)
target_link_libraries( ${PROJECT_NAME}_example ${PROJECT_NAME} ${catkin_LIBRARIES} )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test_cnr_logger.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${rostest_LIBRARIES} )

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()



## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".git" EXCLUDE
 )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## LINT
set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int,--max-line-length=180")
