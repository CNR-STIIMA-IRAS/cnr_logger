
# README #

## CI ##

service    | Kinetic | Melodic | Master
---------- | ------- | ------- | ------
Travis     | [![Build Status](https://travis-ci.org/CNR-STIIMA-IRAS/cnr_logger.svg?branch=kinetic-devel)](https://travis-ci.org/CNR-STIIMA-IRAS/cnr_logger) | [![Build Status](https://travis-ci.org/CNR-STIIMA-IRAS/cnr_logger.svg?branch=melodic-devel)](https://travis-ci.org/CNR-STIIMA-IRAS/cnr_logger) | [![Build Status](https://travis-ci.org/CNR-STIIMA-IRAS/cnr_logger.svg?branch=master)](https://travis-ci.org/CNR-STIIMA-IRAS/cnr_logger)
Codecov    | [![codecov](https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_logger/branch/master/graph/badge.svg)](https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_logger) | [![codecov](https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_logger/branch/master/graph/badge.svg)](https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_logger) | [![codecov](https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_logger/branch/master/graph/badge.svg)](https://codecov.io/gh/CNR-STIIMA-IRAS/cnr_logger)

## Aim ##

* The package has been designed to have a logger separated from the standard ros logging functions. It uses the same core library, log4cxx. The main difference consists of that you can enable or disable the logging to screen and/or to file just using a parameter.

## Usage ##

### Dependencies ###

roscpp

### Parameters ###

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

### Class initialization and usage ###

There are two constructors:

```cpp
TraceLogger( const std::string& logger_id )
TraceLogger( const std::string& logger_id, const std::string& param_namespace, const bool star_header )
```

The first does not initialize the instance of the class, and the function `init()` must be called afterwards.
The second initializes the instance of the class.

If the initialization failed, the class superimpose default values unless the user explicitly indicates to not use the default values.

#### Example of usage ####

```cpp
  #include <iostream>
  #include <ros/ros.h>
  #include <cnr_logger/cnr_logger.h>

  std::shared_ptr<cnr_logger::TraceLogger> logger;
  void return_void_ok()
  {
    CNR_TRACE_START(*logger);
    // some stuff
    CNR_RETURN_OK(*logger, void() );
  }

  void return_void_not_ok()
  {
    CNR_TRACE_START(*logger);
    // some stuff
    CNR_RETURN_NOT_OK(*logger, void() );
  }

  void return_bool()
  {
    CNR_TRACE_START(*logger);
    // some stuff
    bool ret = true;
    CNR_RETURN_BOOL(*logger, true );
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

    logger.reset( new cnr_logger::TraceLogger ( "log1", "/") );    // the first parameters is an ID for the logger,
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

#### Utilities with the package ####

* The ANSI Colors are defined as an inline function

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

* The macros to be used within the code are:

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

### Contribution guidelines ###

### Who do I talk to? ###

* nicola.pedrocchi@stiima.cnr.it
