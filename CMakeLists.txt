cmake_minimum_required(VERSION 2.8.3)

project(cnr_logger)

###################
## OPTIONS       ##
###################
option(USE_ROS "ROS SUPPORT" ON)
option(ENABLE_TESTING "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING "ENABLE COVERAGE TESTING" OFF)
option(COMPILE_EXAMPLE "COMPILE THE EXAMPLE" OFF)

if(USE_ROS)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()

###################
## PRELIMINARIES ##
###################
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

IF("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
  set(CMAKE_CXX_FLAGS "-std=c++11")
ELSE()
  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

#################
## DEPENDECIES ##
#################

## common
find_package(Boost REQUIRED COMPONENTS date_time )

if(USE_ROS)

  MESSAGE(STATUS "ROS SUPPORT ENABLED")

  find_package(catkin REQUIRED COMPONENTS roscpp roslint)
  catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

  ## CODE COVERAGE INTEGRATED WITH ROSTEST
  if(ENABLE_TESTING)
    find_package(rostest REQUIRED)

    if(ENABLE_COVERAGE_TESTING)
      find_package(code_coverage REQUIRED)
      APPEND_COVERAGE_COMPILER_FLAGS()
    endif(ENABLE_COVERAGE_TESTING)
  endif(ENABLE_TESTING)


  set(CNR_LOGGER_TARGET_LINK_LIBRARIES  ${catkin_LIBRARIES} )
  set(CNR_LOGGER_TARGET_INCLUDE_DIRS    ${catkin_INCLUDE_DIRS} )
  set(CNR_LOGGER_INSTALL_INCLUDE_DIR    ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  set(CNR_LOGGER_INSTALL_LIB_DIR        ${CATKIN_PACKAGE_LIB_DESTINATION})

  set(ROSLINT_CPP_OPTS
  "--filter=-whitespace/braces,--filter=-runtime/references,-runtime/int,--max-line-length=180")
  roslint_cpp( src/${PROJECT_NAME}/cnr_logger.cpp )

else(USE_ROS)

  MESSAGE(STATUS "ROS SUPPORT DISABLED")

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

  ### YAML-CPP
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

  ### Log4cxx
  find_package(Log4cxx)

  if(ENABLE_TESTING)
    message(STATUS "Enable testing enable_testing")
    enable_testing()
    find_package(GTest REQUIRED)
    
    if(ENABLE_COVERAGE_TESTING)
      set(CMAKE_CXX_FLAGS "-Wno-deprecated-register ${CMAKE_CXX_FLAGS}")
      set(CMAKE_CXX_FLAGS_DEBUG "-Wno-deprecated-register -O0 -g -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS_DEBUG}")
    endif(ENABLE_COVERAGE_TESTING)
  endif(ENABLE_TESTING)

  #add_compile_definitions(ROS_NOT_AVAILABLE)
  add_definitions(-DROS_NOT_AVAILABLE)

  set(CNR_LOGGER_TARGET_LINK_LIBRARIES  ${YAML_CPP_LIBRARIES} ${Log4cxx_LIBRARY})
  set(CNR_LOGGER_TARGET_INCLUDE_DIRS    ${YAML_CPP_INCLUDE_DIRS})
  set(CNR_LOGGER_INSTALL_INCLUDE_DIR    "${CMAKE_INSTALL_PREFIX}/include")
  set(CNR_LOGGER_INSTALL_LIB_DIR        "${CMAKE_INSTALL_PREFIX}/lib")
endif(USE_ROS)

##########################
## TARGETS              ##
##########################
include_directories   (include ${CNR_LOGGER_TARGET_INCLUDE_DIRS} )
add_library           (${PROJECT_NAME} SHARED src/${PROJECT_NAME}/cnr_logger.cpp )
target_link_libraries (${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES})
target_compile_options(${PROJECT_NAME}
  PUBLIC $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
  target_precompile_headers(${PROJECT_NAME} PUBLIC
            <map>
            <string>
            <iostream>
            <log4cxx/logger.h>
            <log4cxx/basicconfigurator.h>
            <log4cxx/propertyconfigurator.h>
            <log4cxx/helpers/exception.h>
            <log4cxx/rollingfileappender.h>
            <log4cxx/consoleappender.h>
            <log4cxx/patternlayout.h> )
endif()

if(COMPILE_EXAMPLE)
  add_executable        (${PROJECT_NAME}_example src/cnr_logger_example.cpp )
  target_link_libraries (${PROJECT_NAME}_example ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES})
endif()

##########################
## Testing              ##
##########################
if(ENABLE_TESTING)

  if(USE_ROS)
    # Test using ROS TIME
    add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test_cnr_logger.cpp)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_definitions(${PROJECT_NAME}_test PUBLIC FORCE_ROS_TIME_USE=1)

    # Test using STD TIME
    add_rostest_gtest(${PROJECT_NAME}_test_std_time test/test_std_time.launch test/test_cnr_logger.cpp)
    target_link_libraries(${PROJECT_NAME}_test_std_time ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_definitions(${PROJECT_NAME}_test_std_time PUBLIC FORCE_ROS_TIME_USE=0)

  else()

    include_directories(${GTEST_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME}_test test/test_cnr_logger.cpp)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES}  gtest pthread)
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
    
    gtest_add_tests(${PROJECT_NAME}_test "${CMAKE_SOURCE_DIR}/test/config/" AUTO)

  endif(USE_ROS)
endif(ENABLE_TESTING)


##########################
## Coverage Testing     ##
##########################
if( ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  if(USE_ROS)
    message(STATUS "The coverage testing is enabled")
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  else()
    ## NOTHING TO DO, SINCE GTEST ALREADY CREATES THE COVERAGE 
    ## IF THE COMPILING OPTIONS ARE SET
  endif()
endif()







#############
## Install ##
#############
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CNR_LOGGER_INSTALL_INCLUDE_DIR}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".git" EXCLUDE
 )

install(TARGETS ${PROJECT_NAME} 
  ARCHIVE DESTINATION ${CNR_LOGGER_INSTALL_LIB_DIR}
  LIBRARY DESTINATION ${CNR_LOGGER_INSTALL_LIB_DIR}
  RUNTIME DESTINATION ${CNR_LOGGER_INSTALL_LIB_DIR}
)
