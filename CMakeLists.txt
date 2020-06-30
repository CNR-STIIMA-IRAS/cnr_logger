cmake_minimum_required(VERSION 3.1)

project(cnr_logger)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS roscpp roslint)
find_package(Boost REQUIRED COMPONENTS date_time )

catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

include_directories( include ${catkin_INCLUDE_DIRS} )

## Add gtest based cpp test target and link libraries
add_library(${PROJECT_NAME} src/cnr_logger.cpp)
roslint_cpp( src/cnr_logger.cpp )
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
 add_executable(${PROJECT_NAME}-test test/test_cnr_logger.cpp)
 target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

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

set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int,--max-line-length=180")
