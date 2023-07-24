#############################
## PRELIMINARIES           ##
#############################
cmake_minimum_required(VERSION 2.8.12)

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
option(USE_ROS1                 "ROS SUPPORT" ON)
option(ENABLE_TESTING           "ENABLE TESTING" OFF)
option(ENABLE_COVERAGE_TESTING  "ENABLE COVERAGE TESTING" ON)
option(COMPILE_EXAMPLE          "COMPILE THE EXAMPLE" OFF)

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

if(USE_ROS1)

  MESSAGE(STATUS "ROS SUPPORT ENABLED")

  find_package(catkin REQUIRED COMPONENTS roscpp)
  catkin_package( INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} )

  set(CNR_LOGGER_TARGET_LINK_LIBRARIES  ${catkin_LIBRARIES} )
  set(CNR_LOGGER_TARGET_INCLUDE_DIRS    ${catkin_INCLUDE_DIRS} )

else(USE_ROS1)

  MESSAGE(STATUS "ROS SUPPORT DISABLED")

  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

  ### YAML-CPP
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

  ### Threads
  find_package(Threads REQUIRED)

  add_definitions(-DROS1_NOT_AVAILABLE)

  set(CNR_LOGGER_TARGET_LINK_LIBRARIES  ${YAML_CPP_LIBRARIES} ${Log4cxx_LIBRARIES} ${Boost_LIBRARIES})
  set(CNR_LOGGER_TARGET_INCLUDE_DIRS    ${YAML_CPP_INCLUDE_DIRS})

endif(USE_ROS1)

cnr_install_directories(USE_ROS1 
  CNR_LOGGER_INSTALL_INCLUDE_DIR CNR_LOGGER_INSTALL_LIB_DIR CNR_LOGGER_INSTALL_BIN_DIR CNR_LOGGER_INSTALL_SHARE_DIR)
cnr_enable_testing(ENABLE_TESTING ENABLE_COVERAGE USE_ROS1)

##########################
## TARGETS              ##
##########################
set(BUILD_INTERFACE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include ${CNR_LOGGER_TARGET_INCLUDE_DIRS})
set(INSTALL_INTERFACE_INCLUDE_DIRS include ${CNR_LOGGER_TARGET_INCLUDE_DIRS})
set(CNR_LOGGER_LIB_SRCS src/${PROJECT_NAME}/cnr_logger.cpp)

add_library(${PROJECT_NAME} SHARED ${CNR_LOGGER_LIB_SRCS} )
target_link_libraries(${PROJECT_NAME} ${CNR_LOGGER_TARGET_LINK_LIBRARIES})
if("${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0.2)
  include_directories("${BUILD_INTERFACE_INCLUDE_DIRS}")
else()
  target_include_directories(${PROJECT_NAME} PUBLIC 
                              "$<BUILD_INTERFACE:${BUILD_INTERFACE_INCLUDE_DIRS}>"
                                "$<INSTALL_INTERFACE:${INSTALL_INTERFACE_INCLUDE_DIRS}>" )
endif()

cnr_target_compile_options(${PROJECT_NAME})

if(COMPILE_EXAMPLE)
  add_executable(${PROJECT_NAME}_example src/cnr_logger_example.cpp )
  target_link_libraries(${PROJECT_NAME}_example ${PROJECT_NAME})
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
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING  PATTERN "*.h"
                    PATTERN "*.hpp"
                    PATTERN ".svn" EXCLUDE
  )
  
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    PATTERN ".svn" EXCLUDE)

else()

  install(
    DIRECTORY include/
    DESTINATION include
  )

  install(
    TARGETS cnr_logger
    EXPORT export_cnr_logger
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # export targets
  set(export_targets ${export_targets};cnr_logger)
  export(EXPORT export_cnr_logger
    FILE "${PROJECT_BINARY_DIR}/export_cnr_logger.cmake")

  # Create the cnr_loggerConfig.cmake
  set(CONF_INCLUDE_DIRS "${CMAKE_INSTALL_PREFIX}/include")
  configure_file(cnr_loggerConfig.cmake.in
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/cnr_loggerConfig.cmake" @ONLY)

  # Create the cnr_loggerConfigVersion.cmake
  configure_file(cnr_loggerConfigVersion.cmake.in
    "${PROJECT_BINARY_DIR}/cnr_loggerConfigVersion.cmake" @ONLY)

  install(FILES
    "${PROJECT_BINARY_DIR}/${CMAKE_FILES_DIRECTORY}/cnr_loggerConfig.cmake"
    "${PROJECT_BINARY_DIR}/cnr_loggerConfigVersion.cmake"
    
    DESTINATION "share/cnr_logger/cmake" COMPONENT dev)
  install(EXPORT export_cnr_logger
    DESTINATION "share/cnr_logger/cmake"
    FILE export_cnr_logger.cmake
    COMPONENT dev)

endif()
#############################
## END INSTALL             ##
#############################
