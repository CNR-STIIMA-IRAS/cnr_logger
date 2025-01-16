#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 3.16.0)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

get_project_name("package.xml" extracted_name extracted_version)

if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0.2)
  project(${extracted_name})
else()
  project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
endif()

cnr_set_flags()
#############################
## END - PRELIMINARIES     ##
#############################


#############################
## OPTIONS                 ##
#############################
option(USE_ROS1                 "ROS SUPPORT" OFF)
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" ON)
option(COMPILE_EXAMPLE          "COMPILE THE EXAMPLE" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(USE_ROS1)
  set(ENABLE_TESTING ${CATKIN_ENABLE_TESTING})
endif()
#############################
## END - OPTIONS           ##
#############################




#################
## DEPENDECIES ##
#################

## common
find_package(Boost REQUIRED COMPONENTS date_time filesystem system)

## common
find_package(PkgConfig REQUIRED)
pkg_check_modules(Log4cxx REQUIRED liblog4cxx IMPORTED_TARGET)

string(REPLACE "." ";" VERSION_LIST ${Log4cxx_VERSION})
list(GET VERSION_LIST 0 Log4cxx_MAJOR_VERSION)
list(GET VERSION_LIST 1 Log4cxx_MINOR_VERSION)
list(GET VERSION_LIST 2 Log4cxx_PATCH_VERSION)

add_definitions(-DLog4cxx_MAJOR_VERSION=${Log4cxx_MAJOR_VERSION})
add_definitions(-DLog4cxx_MINOR_VERSION=${Log4cxx_MINOR_VERSION})
add_definitions(-DLog4cxx_PATCH_VERSION=${Log4cxx_PATCH_VERSION})


### YAML-CPP
# find_package(PkgConfig REQUIRED)
# pkg_check_modules(yaml-cpp_pkg_config REQUIRED yaml-cpp IMPORTED_TARGET)

find_package(yaml-cpp REQUIRED)
if(yaml-cpp VERSION_LESS "0.8")
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp IMPORTED_TARGET GLOBAL)

  # pkg_check_modules(YAML_CPP REQUIRED yaml-cpp IMPORTED_TARGET GLOBAL)
  add_library(yaml-cpp::yaml-cpp ALIAS PkgConfig::YAML_CPP)
endif()

### Threads
find_package(Threads REQUIRED)


if(USE_ROS1)

  MESSAGE(STATUS "ROS SUPPORT ENABLED")

  find_package(catkin REQUIRED COMPONENTS roscpp)
  catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

  set(CNR_LOGGER_TARGET_LINK_LIBRARIES  ${catkin_LIBRARIES} )
  set(CNR_LOGGER_TARGET_INCLUDE_DIRS    ${catkin_INCLUDE_DIRS} )

else(USE_ROS1)

  MESSAGE(STATUS "ROS SUPPORT DISABLED")

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)



  add_definitions(-DROS1_NOT_AVAILABLE)

  set(CNR_LOGGER_TARGET_LINK_LIBRARIES  "")
  set(CNR_LOGGER_TARGET_INCLUDE_DIRS    "")

endif(USE_ROS1)

cnr_install_directories(USE_ROS1 
  CNR_LOGGER_INSTALL_INCLUDE_DIR CNR_LOGGER_INSTALL_LIB_DIR CNR_LOGGER_INSTALL_BIN_DIR CNR_LOGGER_INSTALL_SHARE_DIR)
cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)

if (ENABLE_TESTING)
  find_package(GTest REQUIRED)
  include(GoogleTest)
  include(CTest)
endif()

##########################
## TARGETS              ##
##########################
set(BUILD_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include ${CNR_LOGGER_TARGET_INCLUDE_DIRS})
set(INSTALL_INTERFACE_INCLUDE_DIRS include ${CNR_LOGGER_TARGET_INCLUDE_DIRS})
set(CNR_LOGGER_LIB_SRCS src/${PROJECT_NAME}/cnr_logger.cpp)

add_library(${PROJECT_NAME} SHARED ${CNR_LOGGER_LIB_SRCS} )
list(APPEND TARGETS_LIST ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}
  PUBLIC
  yaml-cpp::yaml-cpp
  PkgConfig::Log4cxx
  PUBLIC Boost::system
  PUBLIC Boost::filesystem
  Threads::Threads
  ${CNR_LOGGER_TARGET_LINK_LIBRARIES})
add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})


if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0.2)
  include_directories("${BUILD_INTERFACE_INCLUDE_DIRS}")
else()
  target_include_directories(${PROJECT_NAME} PUBLIC 
                              "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
                                "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>" )
endif()

# Alias for library
add_library("${PROJECT_NAME}::${PROJECT_NAME}" ALIAS ${PROJECT_NAME})

cnr_target_compile_options(${PROJECT_NAME})

if(COMPILE_EXAMPLE)
  add_executable(${PROJECT_NAME}_example src/cnr_logger_example.cpp )
  target_link_libraries(${PROJECT_NAME}_example "${PROJECT_NAME}::${PROJECT_NAME}")
  cnr_target_compile_options(${PROJECT_NAME}_example)

  list(APPEND TARGETS_LIST ${PROJECT_NAME}_example)

endif()

##########################
## Testing              ##
##########################
if(ENABLE_TESTING)
  if(USE_ROS1)

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
      target_link_libraries(${PROJECT_NAME}_test "${PROJECT_NAME}::${PROJECT_NAME}" GTest::gtest_main Threads::Threads)
    else()
    target_link_libraries(${PROJECT_NAME}_test "${PROJECT_NAME}::${PROJECT_NAME}" GTest::gtest_main)
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

  endif(USE_ROS1)

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
  if(USE_ROS1)
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



#############################
## INSTALL                 ##
#############################
if(USE_ROS1)
  cnr_install_catkin_policy(${TARGETS_LIST})
else()
  set(PROJECT_VERSION 1.0.0)
  cnr_install_cmake_policy(
      ${CNR_LOGGER_INSTALL_INCLUDE_DIR} 
      ${CNR_LOGGER_INSTALL_LIB_DIR}
      ${CNR_LOGGER_INSTALL_BIN_DIR}
      ${extracted_version} 
      ${TARGETS_LIST}
  )
endif()
#############################
## END INSTALL             ##
#############################
