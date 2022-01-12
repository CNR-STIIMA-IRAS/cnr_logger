cmake_minimum_required(VERSION 2.8.3)

project(cnr_logger)

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

option(USE_ROS "ROS SUPPORT" ON)

if(NOT USE_ROS)

  MESSAGE(STATUS "ROS SUPPORT DISABLED")
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

  find_package(Log4cxx)
  
  add_compile_definitions(ROS_NOT_AVAILABLE)

  set(CNR_LOGGER_TARGET_LINK_LIBRARIES ${YAML_CPP_LIBRARIES} ${Log4cxx_LIBRARY})

  include_directories( include ${YAML_CPP_INCLUDE_DIRS} )

  set(CNR_LOGGER_INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include")
  set(CNR_LOGGER_INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib")

else()

  MESSAGE(STATUS "ROS SUPPORT ENABLED")
  find_package(catkin REQUIRED COMPONENTS roscpp roslint)
  set(CNR_LOGGER_TARGET_LINK_LIBRARIES ${catkin_LIBRARIES})
  include_directories( include ${catkin_INCLUDE_DIRS} )
  catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

  set(ROSLINT_CPP_OPTS
    "--filter=-whitespace/braces,--filter=-runtime/references,-runtime/int,--max-line-length=180")
  roslint_cpp( src/${PROJECT_NAME}/cnr_logger.cpp )

  set(CNR_LOGGER_INSTALL_INCLUDE_DIR ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  set(CNR_LOGGER_INSTALL_LIB_DIR ${CATKIN_PACKAGE_LIB_DESTINATION})

endif(NOT USE_ROS)


find_package(Boost REQUIRED COMPONENTS date_time )

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_logger.cpp )
target_link_libraries( ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES})
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

#############
## Testing ##
#############
if(USE_ROS)
  ## Add gtest based cpp test target and link libraries
  if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test_cnr_logger.cpp)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_definitions(${PROJECT_NAME}_test PUBLIC FORCE_ROS_TIME_USE=1)

    add_rostest_gtest(${PROJECT_NAME}_test_std_time test/test_std_time.launch test/test_cnr_logger.cpp)
    target_link_libraries(${PROJECT_NAME}_test_std_time ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES} ${rostest_LIBRARIES} )
    target_compile_definitions(${PROJECT_NAME}_test_std_time PUBLIC FORCE_ROS_TIME_USE=0)

    if(ENABLE_COVERAGE_TESTING)
      set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
      add_code_coverage(
        NAME ${PROJECT_NAME}_coverage_report
        DEPENDENCIES ${PROJECT_NAME}_test
      )
    endif()
  endif()

#################
else()
#################
  enable_testing()
  find_package(GTest REQUIRED)
  include_directories(${GTEST_INCLUDE_DIRS})

  add_executable(${PROJECT_NAME}_test test/test_cnr_logger.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES}  gtest pthread)
  target_compile_definitions(${PROJECT_NAME}_test PRIVATE TEST_DIR="${CMAKE_CURRENT_LIST_DIR}/test")
  
  gtest_add_tests(${PROJECT_NAME}_test "${CMAKE_SOURCE_DIR}/test/config/" AUTO)
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
