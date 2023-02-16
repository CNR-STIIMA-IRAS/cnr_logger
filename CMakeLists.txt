cmake_minimum_required(VERSION 2.8.3)

project(cnr_logger)

if(${CMAKE_VERSION} VERSION_GREATER  "3.0.0")
  cmake_policy(SET CMP0048 NEW)
  set(cnr_logger_MAJOR_VERSION 0)
  set(cnr_logger_MINOR_VERSION 1)
  set(cnr_logger_PATCH_VERSION 0)
  set(cnr_logger_VERSION
    ${cnr_logger_MAJOR_VERSION}.${cnr_logger_MINOR_VERSION}.${cnr_logger_PATCH_VERSION})
endif()

set(LOCAL_CXX_STANDARD 14)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  if(CMAKE_CXX_COMPILER_VERSION GREATER 12)
    set(LOCAL_CXX_STANDARD 17)
  endif()
elseif(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  if(CMAKE_CXX_COMPILER_VERSION GREATER 10)
    set(LOCAL_CXX_STANDARD 17)
  endif()
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${LOCAL_CXX_STANDARD})
endif()

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

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

cnr_set_flags()

#################
## DEPENDECIES ##
#################

## common
find_package(Boost REQUIRED COMPONENTS date_time filesystem)

## common
find_package(PkgConfig REQUIRED)
pkg_check_modules(Log4cxx REQUIRED liblog4cxx)

string(REPLACE "." ";" VERSION_LIST ${Log4cxx_VERSION})
list(GET VERSION_LIST 0 Log4cxx_MAJOR_VERSION)
list(GET VERSION_LIST 1 Log4cxx_MINOR_VERSION)
list(GET VERSION_LIST 2 Log4cxx_PATCH_VERSION)

add_definitions(-DLog4cxx_MAJOR_VERSION=${Log4cxx_MAJOR_VERSION})
add_definitions(-DLog4cxx_MINOR_VERSION=${Log4cxx_MINOR_VERSION})
add_definitions(-DLog4cxx_PATCH_VERSION=${Log4cxx_PATCH_VERSION})

if(USE_ROS)

  MESSAGE(STATUS "ROS SUPPORT ENABLED")

  find_package(catkin REQUIRED COMPONENTS roscpp roslint)
  catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

  ## CODE COVERAGE INTEGRATED WITH ROSTEST
  if(ENABLE_TESTING)
    find_package(rostest REQUIRED)

    if(ENABLE_COVERAGE_TESTING AND NOT WIN32)
      find_package(code_coverage REQUIRED)
      APPEND_COVERAGE_COMPILER_FLAGS()
    endif(ENABLE_COVERAGE_TESTING AND NOT WIN32)
  endif(ENABLE_TESTING)

  set(CNR_LOGGER_TARGET_LINK_LIBRARIES  ${catkin_LIBRARIES} )
  set(CNR_LOGGER_TARGET_INCLUDE_DIRS    ${catkin_INCLUDE_DIRS} )

  set(CNR_LOGGER_INSTALL_INCLUDE_DIR    ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  set(CNR_LOGGER_INSTALL_LIB_DIR        ${CATKIN_PACKAGE_LIB_DESTINATION})
  set(CNR_LOGGER_INSTALL_BIN_DIR        ${CATKIN_GLOBAL_BIN_DESTINATION})
  set(CNR_LOGGER_INSTALL_SHARE_DIR      ${CATKIN_PACKAGE_SHARE_DESTINATION})

  set(ROSLINT_CPP_OPTS
  "--filter=-whitespace/braces,--filter=-runtime/references,-runtime/int,--max-line-length=180")
  roslint_cpp( src/${PROJECT_NAME}/cnr_logger.cpp )

else(USE_ROS)

  MESSAGE(STATUS "ROS SUPPORT DISABLED")

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

  ### YAML-CPP
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

  ### Threads
  find_package(Threads REQUIRED)

  if(ENABLE_TESTING)
    message(STATUS "Enable testing enable_testing")
    enable_testing()
    find_package(GTest REQUIRED)

    if(ENABLE_COVERAGE_TESTING AND NOT WIN32)
      set(CMAKE_CXX_FLAGS "-Wno-deprecated-register ${CMAKE_CXX_FLAGS}")
      set(CMAKE_CXX_FLAGS_DEBUG "-Wno-deprecated-register -O0 -g -fprofile-arcs -ftest-coverage ${CMAKE_CXX_FLAGS_DEBUG}")
    endif(ENABLE_COVERAGE_TESTING AND NOT WIN32)
  endif(ENABLE_TESTING)

  #add_compile_definitions(ROS_NOT_AVAILABLE)
  add_definitions(-DROS_NOT_AVAILABLE)

  set(CNR_LOGGER_TARGET_LINK_LIBRARIES  ${YAML_CPP_LIBRARIES} ${Log4cxx_LIBRARIES} ${Boost_LIBRARIES})
  set(CNR_LOGGER_TARGET_INCLUDE_DIRS    ${YAML_CPP_INCLUDE_DIRS})

  set(CNR_LOGGER_INSTALL_INCLUDE_DIR    "${CMAKE_INSTALL_PREFIX}/include")
  set(CNR_LOGGER_INSTALL_LIB_DIR        "${CMAKE_INSTALL_PREFIX}/lib")
  set(CNR_LOGGER_INSTALL_BIN_DIR        "${CMAKE_INSTALL_PREFIX}/bin")
  set(CNR_LOGGER_INSTALL_SHARE_DIR      "${CMAKE_INSTALL_PREFIX}/share")
endif(USE_ROS)

##########################
## TARGETS              ##
##########################
include_directories   (include ${CNR_LOGGER_TARGET_INCLUDE_DIRS} )
add_library           (${PROJECT_NAME} SHARED src/${PROJECT_NAME}/cnr_logger.cpp )
target_link_libraries (${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES})
#cnr_target_compile_options(${PROJECT_NAME})

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
  cnr_target_compile_options(${PROJECT_NAME}_example)

  install(TARGETS ${PROJECT_NAME}_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

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
    cnr_target_compile_options(${PROJECT_NAME}_test)

    # Test using STD TIME
    add_rostest_gtest(${PROJECT_NAME}_test_std_time test/test_std_time.launch test/test_cnr_logger.cpp)
    target_link_libraries(${PROJECT_NAME}_test_std_time ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_definitions(${PROJECT_NAME}_test_std_time PUBLIC FORCE_ROS_TIME_USE=0)
    cnr_target_compile_options(${PROJECT_NAME}_test_std_time)

  else()

    include_directories(${GTEST_INCLUDE_DIRS})
    add_executable(${PROJECT_NAME}_test test/test_cnr_logger.cpp)
    if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
      target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES} gtest Threads::Threads)
    else()
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES} gtest)
      if(THREADS_HAVE_PTHREAD_ARG)
        target_compile_options(${PROJECT_NAME}_test PUBLIC "-pthread")
      endif()
      if(CMAKE_THREAD_LIBS_INIT)
        target_link_libraries(${PROJECT_NAME}_test "${CMAKE_THREAD_LIBS_INIT}")
      endif()
    endif()
    target_compile_definitions(${PROJECT_NAME}_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
    cnr_target_compile_options(${PROJECT_NAME}_test)

    gtest_add_tests(${PROJECT_NAME}_test "${CMAKE_SOURCE_DIR}/test/config/" AUTO)

  endif(USE_ROS)

  install(TARGETS ${PROJECT_NAME}_test
    ARCHIVE DESTINATION ${CNR_LOGGER_INSTALL_SHARE_DIR}
    LIBRARY DESTINATION ${CNR_LOGGER_INSTALL_SHARE_DIR}
    RUNTIME DESTINATION ${CNR_LOGGER_INSTALL_SHARE_DIR})

  install(DIRECTORY test/config
    DESTINATION ${CNR_LOGGER_INSTALL_SHARE_DIR}
    PATTERN ".svn" EXCLUDE)

  install(DIRECTORY test/
    DESTINATION ${CNR_LOGGER_INSTALL_SHARE_DIR}/launch
    PATTERN ".svn" EXCLUDE)

endif(ENABLE_TESTING)


##########################
## Coverage Testing     ##
##########################
if( ENABLE_TESTING AND ENABLE_COVERAGE_TESTING AND NOT WIN32 )
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
 install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CNR_LOGGER_INSTALL_INCLUDE_DIR}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".git" EXCLUDE
 )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CNR_LOGGER_INSTALL_LIB_DIR}
  LIBRARY DESTINATION ${CNR_LOGGER_INSTALL_LIB_DIR}
  RUNTIME DESTINATION ${CNR_LOGGER_INSTALL_BIN_DIR}
)

if(USE_ROS)
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)
endif()
