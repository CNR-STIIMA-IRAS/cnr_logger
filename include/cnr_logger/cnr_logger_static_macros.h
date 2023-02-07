/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2021 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file cnr_logger_macros.h
 * @author Nicola Pedrocchi, ALessio Prini
 * @date 30 Gen 2021
 * @brief File containing the macro definition.
 *
 * The macro have been designed to follow the basic log4cxx structure (and ROS).
 * The further macro CNR_TRACE_START and CNR_RETURN_xxx are used to trace the input and the output values of the functions
 */
#ifndef CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_H
#define CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_H

#include <iostream>
#include <type_traits>
#include <cstdint>
#include <string>

#include <cnr_logger/cnr_logger.h>

// user can force the use of ROS time by adding "-DFORCE_ROS_TIME_USE"

#if defined(FORCE_ROS_TIME_USE)
  #include <ros/time.h>
  #include <ros/console.h>
  #define CONSOLE_THROTTLE_CHECK(now, last, period)\
    ROSCONSOLE_THROTTLE_CHECK(now, last, period)

  #define TIME_NOW()\
    ::ros::Time::now().toSec()
#else
  #include <ctime>
  #if defined(_MSC_VER)
  #define ROS_LIKELY(x)       (x)
  #define ROS_UNLIKELY(x)     (x)
  #else
  #define ROS_LIKELY(x)       __builtin_expect((x),1)
  #define ROS_UNLIKELY(x)     __builtin_expect((x),0)
  #endif

  #define CONSOLE_THROTTLE_CHECK(now, last, period)\
    (ROS_UNLIKELY(last + period <= now) || ROS_UNLIKELY(now < last))

  #define TIME_NOW()\
   std::time(0)
#endif



namespace cnr_logger
{
    
inline cnr_logger::TraceLogger* getTraceLogger(TraceLogger* logger)
{
  return logger;
}

inline cnr_logger::TraceLogger* getTraceLogger(TraceLogger& logger)
{
  return &logger;
}

inline cnr_logger::TraceLogger* getTraceLogger(TraceLoggerPtr logger)
{
  if (logger)
  {
    return logger.get();
  }
  else
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__
                <<": The logger is not initialized. Fake log enabled." << std::endl;
    auto ret = std::make_shared<TraceLogger>();
    return ret.get();
  }
}




}  // namespace cnr_logger


//! Shortcut
namespace cl = cnr_logger;


// ================= STANDARD
#define CNR_FATAL(logger, args)\
do \
{ \
  if(cl::getTraceLogger(logger) && cl::getTraceLogger(logger)->logFatal())\
  {\
    if(cl::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_FATAL(cl::getTraceLogger(logger)->syncFileAndScreenLogger(), args)\
    if(cl::getTraceLogger(logger)->logOnlyFile())\
        LOG4CXX_FATAL(cl::getTraceLogger(logger)->fileLogger(), args)\
    if(cl::getTraceLogger(logger)->logOnlyScreen())\
        LOG4CXX_FATAL(cl::getTraceLogger(logger)->consoleLogger(), args)\
  }\
} while (false)

#define CNR_ERROR(logger, args)\
do \
{\
  if(cl::getTraceLogger(logger) && cl::getTraceLogger(logger)->logError())\
  {\
    if (cl::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_ERROR(cl::getTraceLogger(logger)->syncFileAndScreenLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyFile())\
        LOG4CXX_ERROR(cl::getTraceLogger(logger)->fileLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyScreen())\
        LOG4CXX_ERROR(cl::getTraceLogger(logger)->consoleLogger(), args)\
  }\
} while (false)

#define CNR_WARN(logger, args)\
do\
{\
  if(cl::getTraceLogger(logger) && cl::getTraceLogger(logger)->logWarn())\
  {\
    if (cl::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_WARN(cl::getTraceLogger(logger)->syncFileAndScreenLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyFile())\
        LOG4CXX_WARN(cl::getTraceLogger(logger)->fileLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyScreen())\
        LOG4CXX_WARN(cl::getTraceLogger(logger)->consoleLogger(), args)\
  }\
} while (false)

#define CNR_INFO(logger, args)\
do\
{\
  if(cl::getTraceLogger(logger) && cl::getTraceLogger(logger)->logInfo())\
  {\
    if ((cl::getTraceLogger(logger)->logSyncFileAndScreen())) \
    {\
      LOG4CXX_INFO(cl::getTraceLogger(logger)->syncFileAndScreenLogger(), args)\
    }\
    if (cl::getTraceLogger(logger)->logOnlyFile()) \
    {\
      LOG4CXX_INFO(cl::getTraceLogger(logger)->fileLogger(), args)\
    }\
    if (cl::getTraceLogger(logger)->logOnlyScreen()) \
    {\
      LOG4CXX_INFO(cl::getTraceLogger(logger)->consoleLogger(), args)\
    }\
  }\
} while (false)

#define CNR_INFO_ONLY_FILE(logger, args)\
do\
{\
  if(cl::getTraceLogger(logger) && cl::getTraceLogger(logger)->logInfo())\
  {\
    if ((cl::getTraceLogger(logger)->logSyncFileAndScreen())) \
      LOG4CXX_INFO(cl::getTraceLogger(logger)->syncFileAndScreenLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyFile()) \
      LOG4CXX_INFO(cl::getTraceLogger(logger)->fileLogger(), args)\
  }\
} while (false)

#define CNR_DEBUG(logger, args)\
do\
{\
  if(cl::getTraceLogger(logger) && cl::getTraceLogger(logger)->logDebug())\
  {\
    if (cl::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_DEBUG(cl::getTraceLogger(logger)->syncFileAndScreenLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyFile())\
      LOG4CXX_DEBUG(cl::getTraceLogger(logger)->fileLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyScreen())\
      LOG4CXX_DEBUG(cl::getTraceLogger(logger)->consoleLogger(), args)\
  }\
} while (false)

#define CNR_TRACE(logger, args)\
do\
{\
  if(cl::getTraceLogger(logger) && cl::getTraceLogger(logger)->logTrace())\
  {\
    if (cl::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_TRACE(cl::getTraceLogger(logger)->syncFileAndScreenLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyFile())\
      LOG4CXX_TRACE(cl::getTraceLogger(logger)->fileLogger(), args)\
    if (cl::getTraceLogger(logger)->logOnlyScreen())\
      LOG4CXX_TRACE(cl::getTraceLogger(logger)->consoleLogger(), args)\
  }\
} while (false)
// =================


// ================= THROTTLE
#define CNR_FATAL_THROTTLE(logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = TIME_NOW(); \
    if (CONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_FATAL(logger, args); \
    } \
  } while (false)

#define CNR_ERROR_THROTTLE(logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = TIME_NOW(); \
    if (CONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_ERROR(logger, args); \
    } \
  } while (false)

#define CNR_WARN_THROTTLE(logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = TIME_NOW(); \
    if (CONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_WARN(logger, args); \
    } \
  } while (false)

#define CNR_INFO_THROTTLE(logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = TIME_NOW(); \
    if (CONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_INFO(logger, args); \
    } \
  } while (false)

#define CNR_DEBUG_THROTTLE(logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = TIME_NOW(); \
    if (CONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_DEBUG(logger, args); \
    } \
  } while (false)


#define CNR_TRACE_THROTTLE(logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = TIME_NOW(); \
    if (CONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_TRACE(logger, args); \
    } \
  } while (false)
// =================


// ================= COND
#define CNR_FATAL_COND(logger, cond, args)\
  do\
  {\
    if (cond) { CNR_FATAL(logger, args); }\
  } while (false)

#define CNR_ERROR_COND(logger, cond, args)\
  do\
  {\
    if (cond) { CNR_ERROR(logger, args); }\
  } while (false)

#define CNR_WARN_COND(logger, cond, args)\
  do\
  {\
    if (cond) { CNR_WARN(logger, args); }\
} while (false)

#define CNR_INFO_COND(logger, cond, args)\
  do\
  {\
    if (cond) { CNR_INFO(logger, args); }\
  } while (false)

#define CNR_DEBUG_COND(logger, cond, args)\
  do\
  {\
    if (cond) { CNR_DEBUG(logger, args); }\
  } while (false)

#define CNR_TRACE_COND(logger, cond, args)\
  do\
  {\
    if (cond) { CNR_TRACE(logger, args); }\
  } while (false)


// ================= COND THROTTLE
#define CNR_FATAL_COND_THROTTLE(logger, cond, period, args)\
do\
{\
  if (cond) { CNR_FATAL_THROTTLE(logger, period, args); }\
} while (false)

#define CNR_ERROR_COND_THROTTLE(logger, cond, period, args)\
do\
{\
  if (cond) { CNR_ERROR_THROTTLE(logger, period, args); }\
} while (false)

#define CNR_WARN_COND_THROTTLE(logger, cond, period, args)\
do\
{\
  if (cond) { CNR_WARN_THROTTLE(logger, period, args); }\
} while (false)

#define CNR_INFO_COND_THROTTLE(logger, cond, period, args)\
do\
{\
  if (cond) { CNR_INFO_THROTTLE(logger, period, args); }\
} while (false)

#define CNR_DEBUG_COND_THROTTLE(logger, cond, period, args)\
do\
{\
  if (cond) { CNR_DEBUG_THROTTLE(logger, period, args); }\
} while (false)

#define CNR_TRACE_COND_THROTTLE(logger, cond, period, args)\
do\
{\
  if (cond) { CNR_TRACE_THROTTLE(logger, period, args); }\
} while (false)


// =============================== RETURN UTILS
#define CNR_RETURN_OK(logger, var, msg)\
do\
{\
  std::string _msg = msg;\
  CNR_INFO_COND(logger, _msg.length() > 0, _msg);\
  CNR_TRACE(logger, "[  DONE] " << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_NOTOK(logger, var, msg)\
do\
{\
  std::string _msg = msg;\
  CNR_ERROR_COND(logger, _msg.length(), _msg);\
  CNR_TRACE(logger, cl::RED() + "[FAILED] " + cl::RESET() << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_VOID(logger, ok, msg)\
do\
{\
  std::string _msg = msg;\
  if(ok)\
  {\
    CNR_RETURN_OK(logger,void(),msg);\
  }\
  else\
  {\
    CNR_RETURN_NOTOK(logger,void(),msg);\
  }\
} while (false);\
return;


#define CNR_RETURN_OK_THROTTLE(logger, var, period, msg)\
do\
{\
  std::string _msg = msg;\
  CNR_INFO_COND_THROTTLE(logger, _msg.length() > 0, period, _msg);\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, "[  DONE] " << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_NOTOK_THROTTLE(logger, var, period, msg)\
do\
{\
  std::string _msg = msg;\
  CNR_ERROR_COND_THROTTLE(logger, _msg.length() > 0, period, _msg);\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, cl::RED() + "[FAILED] " + cl::RESET() << __FUNCTION__);\
} while (false);\
return(var);


#define CNR_RETURN_OK_THROTTLE_DEFAULT(logger, var, msg)\
do\
{\
  std::string _msg = msg;\
  double period = cl::getTraceLogger(logger) ? cl::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_INFO_COND_THROTTLE(logger, _msg.length() > 0, period, _msg);\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, "[  DONE] " << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_NOTOK_THROTTLE_DEFAULT(logger, var, msg)\
do\
{\
  std::string _msg = msg;\
  double period = cl::getTraceLogger(logger) ? cl::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_ERROR_COND_THROTTLE(logger, _msg.length() > 0, period, _msg);\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, cl::RED() + "[FAILED] " + cl::RESET() << __FUNCTION__);\
} while (false);\
return(var);

#endif // CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_H
