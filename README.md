# cnr_logger #

[![Build Status - Plain CMake][p]][0]
[![Build Status - ROS Toolchain][t]][1]
[![codecov][c]][2] 
[![Codacy Badge][y]][3]
[![FOSSA Status][f]][4]

## Aim ##

The package has been designed to have a logger separated from the standard ros logging functions. It uses the same core library, log4cxx. The main difference consists of that you can enable or disable the logging to screen and/or to file just using a parameter.

## Usage ##

### Dependencies and Building ###

The package is based on [Log4cxx](https://logging.apache.org/log4cxx/latest_stable/), and it can be built on top of [ros](www.ros.org) distribution, or having installed  [yaml-cpp](https://yaml-cpp.docsforge.com/)

The configuration is done through CMake. Enabling the cmake option `USE_ROS`, the package looks for the `yaml-cpp` (the FindLog4cxx.cmake is [here](./cmake/FindLog4cxx.cmake)).

```cmake
option(USE_ROS "ROS ENABLED" OFF)

if(NOT USE_ROS)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
  find_package(yaml-cpp REQUIRED)
  find_package(Log4cxx)
  ...
else()
  find_package(catkin REQUIRED COMPONENTS roscpp roslint)
  ...
endif()
```

#### User Compilation Defitions ####

By default, the time-tag used in the logging is the system time. 
Define the compiler macro `FORCE_ROS_TIME_USE` in the project that links `cnr_logger` to enable the usage of ROS time. 

### Parameters ###

The parameters can be stored in a `file.yaml` or in the `rosparam server` according to the package building modality.

```yaml
  ~/appenders: ['file', 'screen']                 # Mandatory
                                                  # A vector od dimension 1 or 2, where you can select if the output will be streamed to file, to screen or to both
                                                  # (the order in the vector is not important)

  ~/levels: [ debug|info|warn|error|fatal, ..]    # Optional
                                                  # A vector where you can select the verbosity level for each appender.
                                                  # If not present, or if the size of the vector is different from the dimension of the appenders,
                                                  # the default value is superimposed.
                                                  # Default: 'debug' for all the appenders

  ~/pattern_layout: '...'                         # Optional
                                                  # look at https://svn.apache.org/repos/asf/logging/site/trunk/docs/log4cxx/apidocs/classlog4cxx_1_1_pattern_layout.html"
                                                  # This allows you to define the log pattern.
                                                  # Default is: [%5p] [%d{HH:mm:ss,SSS}][%r][%M:%L]: %m%n

  ~/file_name: 'file_name'                        # Optional
                                                  # If 'file' is selected, this is the path of the log file.
                                                  # If any absolute path is indicated it saves under the default location.
                                                  # Default: ~/.ros/log/[logger_id].log

  ~/append_date_to_file_name: true|false          # Optional
                                                  # The named file will be appended with the YYMMDD-HH:MM::SS of creation
                                                  # Default: false

  ~/append_to_file: true|false                    # Optional
                                                  # If true, the content is appended to file. The new content starts with a clear header (with data and start time).
                                                  # If not, the log file is overwritten.
                                                  # Default: true
```

### Class initialization ###

There are two constructors:

```cpp
TraceLogger( )  //default foo ctor, call init( ) afterwards
TraceLogger(const std::string& logger_id,
              const std::string& path,
                const bool star_header = true,
                  const bool default_values = true )
```

The first one does not initialize the instance of the class, and the function `init()` must be called afterwards.
The second one initializes the instance of the class.

The path can be or the full namespace where the parameters are stored in the `rosparam server`, or the full path of the `file.yaml`.

If the initialization failed, the class superimpose default values unless the user explicitly indicates to not use the default values.

### Example of usage ###

```cpp
  #include <iostream>
  #include <cnr_logger/cnr_logger.h>

  std::shared_ptr<cnr_logger::TraceLogger> logger;
  void return_void_ok()
  {
    CNR_TRACE_START(logger);    // the macro accept both object and shared_ptr<TraceLogger>
    // some stuff
    CNR_RETURN_OK(logger, void() );
  }

  void return_void_not_ok()
  {
    CNR_TRACE_START(*logger);
    // some stuff
    CNR_RETURN_NOT_OK(*logger, void() );
  }

  void return_bool()
  {
    CNR_TRACE_START(logger);
    // some stuff
    bool ret = true;
    CNR_RETURN_BOOL(logger, true );
  }

  void return_true()
  {
    CNR_TRACE_START(*logger);
    // some stuff
    bool ret = true;
    CNR_RETURN_TRUE(*logger );
  }

  int main(int argc, char* argv[] )
  {
    ros::init(argc, argv, "cnr_logger_test" );
    ros::NodeHandle nh;

    logger.reset( new cnr_logger::TraceLogger ("log1", "/") );    // the first parameters is an ID for the logger,
                                                                   // the second parameter is the namespace where
                                                                   // to find the configuration parameters
    while( ros::ok() )
    {
      ROS_INFO("Ciao-ros-info");
      ROS_DEBUG("Ciao-ros-debug");

      LOG4CXX_INFO (*logger,"Ciao-log-1-info");
      LOG4CXX_DEBUG(*logger,"Ciao-log-1-debug");

      ros::spinOnce();
      ros::Duration(1.0).sleep();
    }

    return 1;
  }
```

### Utilities with the package ###

The ANSI Colors are defined as an inline function

```cpp
inline std::string RESET        ( ) { return "\033[0m";         }
inline std::string BLACK        ( ) { return "\033[30m";        }
inline std::string RED          ( ) { return "\033[31m";        }
inline std::string GREEN        ( ) { return "\033[32m";        }
inline std::string YELLOW       ( ) { return "\033[33m";        }
inline std::string BLUE         ( ) { return "\033[34m";        }
inline std::string MAGENTA      ( ) { return "\033[35m";        }
inline std::string CYAN         ( ) { return "\033[36m";        }
inline std::string WHITE        ( ) { return "\033[37m";        }
inline std::string BOLDBLACK    ( ) { return "\033[1m\033[30m"; }
inline std::string BOLDRED      ( ) { return "\033[1m\033[31m"; }
inline std::string BOLDGREEN    ( ) { return "\033[1m\033[32m"; }
inline std::string BOLDYELLOW   ( ) { return "\033[1m\033[33m"; }
inline std::string BOLDBLUE     ( ) { return "\033[1m\033[34m"; }
inline std::string BOLDMAGENTA  ( ) { return "\033[1m\033[35m"; }
inline std::string BOLDCYAN     ( ) { return "\033[1m\033[36m"; }
inline std::string BOLDWHITE    ( ) { return "\033[1m\033[37m"; }
```

The macros to be used within the code are:

```cpp
// ================= STANDARD
#define CNR_FATAL( trace_logger, args)
#define CNR_ERROR( trace_logger, args)
#define CNR_WARN( trace_logger, args)
#define CNR_INFO( trace_logger, args)
#define CNR_INFO_ONLY_FILE( trace_logger, args)
#define CNR_DEBUG( trace_logger, args)
#define CNR_TRACE( trace_logger, args)
// =================


// ================= THROTTLE
#define CNR_FATAL_THROTTLE( trace_logger, period, args)
#define CNR_ERROR_THROTTLE( trace_logger, period, args)
#define CNR_WARN_THROTTLE(  trace_logger, period, args)
#define CNR_INFO_THROTTLE(  trace_logger, period, args)
#define CNR_DEBUG_THROTTLE( trace_logger, period, args)
#define CNR_TRACE_THROTTLE( trace_logger, period, args)

// ================= COND
#define CNR_FATAL_COND( trace_logger, cond, args )
#define CNR_ERROR_COND( trace_logger, cond, args )
#define CNR_WARN_COND(  trace_logger, cond, args )
#define CNR_INFO_COND(  trace_logger, cond, args )
#define CNR_DEBUG_COND( trace_logger, cond, args )
#define CNR_TRACE_COND( trace_logger, cond, args )


// ================= COND THROTTLE
#define CNR_FATAL_COND_THROTTLE( trace_logger, cond, period, args )
#define CNR_ERROR_COND_THROTTLE( trace_logger, cond, period, args )
#define CNR_WARN_COND_THROTTLE(  trace_logger, cond, period, args )
#define CNR_INFO_COND_THROTTLE(  trace_logger, cond, period, args )
#define CNR_DEBUG_COND_THROTTLE( trace_logger, cond, period, args )
#define CNR_TRACE_COND_THROTTLE( trace_logger, cond, period, args )

// ============================== IN/OUT Functions to trace the begin and the end of a function
#define CNR_TRACE_START(  logger, ... )
#define CNR_TRACE_END(    logger, ... )
#define CNR_RETURN_BOOL(  logger, ret, ... )
#define CNR_RETURN_TRUE(  logger,  ... )
#define CNR_RETURN_FALSE( logger, ... )
#define CNR_RETURN_FATAL( logger, ... )
#define CNR_RETURN_OK(    logger, var, ... )
#define CNR_RETURN_NOTOK( logger, var, ...)
#define CNR_EXIT_EX(      logger, ok, ... )

#define CNR_TRACE_START_THROTTLE(  logger, period, ... )
#define CNR_RETURN_BOOL_THROTTLE(  logger, ret, period, ... )
#define CNR_RETURN_TRUE_THROTTLE(  logger, period, ... )
#define CNR_RETURN_FALSE_THROTTLE( logger, period, ... )
#define CNR_RETURN_OK_THROTTLE(    logger, var, period, ... )
#define CNR_RETURN_NOTOK_THROTTLE( logger, var, period, ...)
```

### Contact ###

<mailto:nicola.pedrocchi@stiima.cnr.it>

## License ##
[![FOSSA Status][o]][5]

[//]: # ([t]:https://travis-ci.org/CNR-STIIMA-IRAS/cnr_logger.svg?branch=master)
[//]: # ([1]:https://travis-ci.org/CNR-STIIMA-IRAS/cnr_logger)

[p]:https://github.com/CNR-STIIMA-IRAS/cnr_logger/actions/workflows/build_cmake.yml/badge.svg
[0]:https://github.com/CNR-STIIMA-IRAS/cnr_logger/actions/workflows/build_cmake.yml

[t]:https://github.com/CNR-STIIMA-IRAS/cnr_logger/actions/workflows/industrial_ci_action.yml/badge.svg?branch=master
[1]:https://github.com/CNR-STIIMA-IRAS/cnr_logger/actions/workflows/industrial_ci_action.yml

[c]:https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_logger/branch/master/graph/badge.svg
[2]:https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_logger

[y]:https://api.codacy.com/project/badge/Grade/7f1834c02aa84b959ee9b7529deb48d6
[3]:https://app.codacy.com/gh/CNR-STIIMA-IRAS/cnr_logger?utm_source=github.com&utm_medium=referral&utm_content=CNR-STIIMA-IRAS/cnr_logger&utm_campaign=Badge_Grade_Dashboard

[f]:https://app.fossa.com/api/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Fcnr_logger.svg?type=shield
[4]:https://app.fossa.com/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Fcnr_logger?ref=badge_shield

[o]:https://app.fossa.com/api/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Fcnr_logger.svg?type=large
[5]:https://app.fossa.com/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Fcnr_logger?ref=badge_large
