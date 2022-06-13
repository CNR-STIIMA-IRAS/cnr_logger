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

#pragma once // qtcreator, workaround

#ifndef CNR_LOGGER_CNR_LOGGER_MACROS_H
#define CNR_LOGGER_CNR_LOGGER_MACROS_H

#include <tuple>
#include <string>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <log4cxx/rollingfileappender.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>
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

#ifdef _MSC_VER
#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif
#endif

#ifndef __PRETTY_FUNCTION__
#define __PRETTY_FUNCTION__ __func__
#endif


/**
 * @namespace cnr_logger
 */
namespace cnr_logger
{


inline std::string RESET()
{
  return "\033[0m";
}
inline std::string BLACK()
{
  return "\033[30m";
}
inline std::string RED()
{
  return "\033[31m";
}
inline std::string GREEN()
{
  return "\033[32m";
}
inline std::string YELLOW()
{
  return "\033[33m";
}
inline std::string BLUE()
{
  return "\033[34m";
}
inline std::string MAGENTA()
{
  return "\033[35m";
}
inline std::string CYAN()
{
  return "\033[36m";
}
inline std::string WHITE()
{
  return "\033[37m";
}
inline std::string BOLDBLACK()
{
  return "\033[1m\033[30m";
}
inline std::string BOLDRED()
{
  return "\033[1m\033[31m";
}
inline std::string BOLDGREEN()
{
  return "\033[1m\033[32m";
}
inline std::string BOLDYELLOW()
{
  return "\033[1m\033[33m";
}
inline std::string BOLDBLUE()
{
  return "\033[1m\033[34m";
}
inline std::string BOLDMAGENTA()
{
  return "\033[1m\033[35m";
}
inline std::string BOLDCYAN()
{
  return "\033[1m\033[36m";
}
inline std::string BOLDWHITE()
{
  return "\033[1m\033[37m";
}

inline std::string RST()
{
  return RESET();
}
inline std::string BLK()
{
  return BLACK();
}
inline std::string R()
{
  return RED();
}
inline std::string G()
{
  return GREEN();
}
inline std::string Y()
{
  return YELLOW();
}
inline std::string BLE()
{
  return BLUE();
}
inline std::string M()
{
  return MAGENTA();
}
inline std::string C()
{
  return CYAN();
}
inline std::string W()
{
  return WHITE();
}
inline std::string BBLK()
{
  return BOLDBLACK();
}
inline std::string BR()
{
  return BOLDRED();
}
inline std::string BG()
{
  return BOLDGREEN();
}
inline std::string BY()
{
  return BOLDYELLOW();
}
inline std::string BBLE()
{
  return BOLDBLUE();
}
inline std::string BM()
{
  return BOLDMAGENTA();
}
inline std::string BC()
{
  return BOLDCYAN();
}
inline std::string BW()
{
  return BOLDWHITE();
}

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
    std::shared_ptr<TraceLogger> ret(new TraceLogger());
    return ret.get();
  }
}




}  // namespace cnr_logger


// ================= STANDARD
#define CNR_FATAL(logger, args)\
do \
{ \
  if(cnr_logger::getTraceLogger(logger) && cnr_logger::getTraceLogger(logger)->logFatal())\
  {\
    if(cnr_logger::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_FATAL(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
    if(cnr_logger::getTraceLogger(logger)->logFile())\
        LOG4CXX_FATAL(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
    if(cnr_logger::getTraceLogger(logger)->logScreen())\
        LOG4CXX_FATAL(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
  }\
} while (false)

#define CNR_ERROR(logger, args)\
do \
{\
  if(cnr_logger::getTraceLogger(logger) && cnr_logger::getTraceLogger(logger)->logError())\
  {\
    if (cnr_logger::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_ERROR(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
    if (cnr_logger::getTraceLogger(logger)->logFile())\
        LOG4CXX_ERROR(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
    if (cnr_logger::getTraceLogger(logger)->logScreen())\
        LOG4CXX_ERROR(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
  }\
} while (false)

#define CNR_WARN(logger, args)\
do\
{\
  if(cnr_logger::getTraceLogger(logger) && cnr_logger::getTraceLogger(logger)->logWarn())\
  {\
    if (cnr_logger::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_WARN(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
    if (cnr_logger::getTraceLogger(logger)->logFile())\
        LOG4CXX_WARN(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
    if (cnr_logger::getTraceLogger(logger)->logScreen())\
        LOG4CXX_WARN(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
  }\
} while (false)

#define CNR_INFO(logger, args)\
do\
{\
  if(cnr_logger::getTraceLogger(logger) && cnr_logger::getTraceLogger(logger)->logInfo())\
  {\
    if ((cnr_logger::getTraceLogger(logger)->logSyncFileAndScreen())) \
    {\
      LOG4CXX_INFO(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
    }\
    if (cnr_logger::getTraceLogger(logger)->logFile()) \
    {\
      LOG4CXX_INFO(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
    }\
    if (cnr_logger::getTraceLogger(logger)->logScreen()) \
    {\
      LOG4CXX_INFO(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
    }\
  }\
} while (false)

#define CNR_INFO_ONLY_FILE(logger, args)\
do\
{\
  if(cnr_logger::getTraceLogger(logger) && cnr_logger::getTraceLogger(logger)->logInfo())\
  {\
    if ((cnr_logger::getTraceLogger(logger)->logSyncFileAndScreen())) \
      LOG4CXX_INFO(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
    else if (cnr_logger::getTraceLogger(logger)->logFile()) \
      LOG4CXX_INFO(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
  }\
} while (false)

#define CNR_DEBUG(logger, args)\
do\
{\
  if(cnr_logger::getTraceLogger(logger) && cnr_logger::getTraceLogger(logger)->logDebug())\
  {\
    if (cnr_logger::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_DEBUG(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
    if (cnr_logger::getTraceLogger(logger)->logFile())\
      LOG4CXX_DEBUG(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
    if (cnr_logger::getTraceLogger(logger)->logScreen())\
      LOG4CXX_DEBUG(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
  }\
} while (false)

#define CNR_TRACE(logger, args)\
do\
{\
  if(cnr_logger::getTraceLogger(logger) && cnr_logger::getTraceLogger(logger)->logTrace())\
  {\
    if (cnr_logger::getTraceLogger(logger)->logSyncFileAndScreen())\
      LOG4CXX_TRACE(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
    if (cnr_logger::getTraceLogger(logger)->logFile())\
      LOG4CXX_TRACE(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
    if (cnr_logger::getTraceLogger(logger)->logScreen())\
      LOG4CXX_TRACE(cnr_logger::getTraceLogger(logger)->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
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


//CNR_INFO_COND(logger, (std::string(__VA_ARGS__).length() > 0), std::string(__VA_ARGS__));


// ============================== IN/OUT Functions
#define CNR_TRACE_START(logger, ...)\
do\
{\
  if((std::string(__VA_ARGS__).length() > 0))\
    CNR_TRACE(logger, cnr_logger::YELLOW() + "[ START] " +cnr_logger::RESET() << std::string(__VA_ARGS__) );\
  else\
    CNR_TRACE(logger, cnr_logger::YELLOW() + "[ START] " +cnr_logger::RESET() << __FUNCTION__);\
} while (false)

#define CNR_TRACE_END(logger, ...)\
do\
{\
  CNR_INFO_COND(logger, (std::string(__VA_ARGS__).length() > 0), std::string(__VA_ARGS__)); \
  CNR_TRACE(logger, "[  DONE] " << __FUNCTION__);\
} while (false)


#define CNR_RETURN_BOOL(logger, ret, ...)\
do\
{\
  std::string result = (ret ? cnr_logger::GREEN()+"[  DONE] " : cnr_logger::RED()+"[FAILED] ") + cnr_logger::RESET();\
  if((std::string(__VA_ARGS__).length() > 0))\
    CNR_TRACE(logger, result << std::string(__VA_ARGS__) );\
  else\
    CNR_TRACE(logger, result << __FUNCTION__);\
} while (false);\
return ret;\


#define CNR_RETURN_TRUE(logger,  ...)\
do\
{\
  if(std::string(__VA_ARGS__).length()>0)\
  {\
    CNR_RETURN_BOOL(logger, true, __VA_ARGS__);\
  }\
  else\
  {\
    CNR_RETURN_BOOL(logger, true,"");\
  }\
} while (false);\

#define CNR_RETURN_FALSE(logger, ...)\
do\
{\
  if(std::string(__VA_ARGS__).length()>0)\
  {\
    CNR_RETURN_BOOL(logger, false, __VA_ARGS__);\
  }\
  else\
  {\
    CNR_RETURN_BOOL(logger, false,"");\
  }\
} while (false);\

#define CNR_RETURN_FATAL(logger, ...)\
  do\
  {\
    CNR_FATAL_COND(logger, std::string(__VA_ARGS__).length() > 0, std::string(__VA_ARGS__));\
    CNR_TRACE(logger, (cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET())<< __FUNCTION__);\
  } while (false);\
  return false;\


#define CNR_RETURN_OK(logger, var, ...)\
do\
{\
  CNR_INFO_COND(logger, std::string(__VA_ARGS__).length() > 0, std::string(__VA_ARGS__));\
  CNR_TRACE(logger, "[  DONE] " << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_NOTOK(logger, var, ...)\
do\
{\
  CNR_ERROR_COND(logger, std::string(__VA_ARGS__).length() > 0, std::string(__VA_ARGS__));\
  CNR_TRACE(logger, cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET() << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_EXIT_EX(logger, ok, ...)\
do\
{\
  if (ok) \
  {\
    CNR_ERROR_COND(logger, std::string(__VA_ARGS__).length() > 0, std::string(__VA_ARGS__)); \
    CNR_TRACE(logger, "[  DONE] " << __FUNCTION__);\
    return;\
  }\
  else\
  {\
    CNR_INFO_COND(logger, std::string(__VA_ARGS__).length() > 0, std::string(__VA_ARGS__));\
    CNR_TRACE(logger, cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET() << __FUNCTION__);\
    throw std::invalid_argument(std::string(__VA_ARGS__).c_str());\
  }\
} while (false)


// ============================== IN/OUT Functions THROTTLE
#define CNR_TRACE_START_THROTTLE(logger, period, ...)\
do\
{\
  CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__)); \
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, "[ START] " << __FUNCTION__);\
} while (false)

#define CNR_RETURN_BOOL_THROTTLE(logger, ret, period, ...)\
do\
{\
  if (ret) { CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__)); }\
  else     { CNR_ERROR_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));}\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, (ret ? "[  DONE] " : cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET())\
                                    << __FUNCTION__);\
} while (false);\
return ret;

#define CNR_RETURN_TRUE_THROTTLE(logger,  period, ...)\
    CNR_RETURN_BOOL_THROTTLE(logger, true, period, __VA_ARGS__);

#define CNR_RETURN_FALSE_THROTTLE(logger, period, ...)\
  CNR_RETURN_BOOL_THROTTLE(logger, false, period, __VA_ARGS__);

#define CNR_RETURN_OK_THROTTLE(logger, var, period, ...)\
do\
{\
  CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, "[  DONE] " << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_NOTOK_THROTTLE(logger, var, period, ...)\
do\
{\
  CNR_ERROR_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET() << __FUNCTION__);\
} while (false);\
return(var);




// ============================== IN/OUT Functions THROTTLE DEFAULT
#define CNR_TRACE_START_THROTTLE_DEFAULT(logger, ...)\
do\
{\
  double period = cnr_logger::getTraceLogger(logger) ? cnr_logger::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__)); \
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, "[ START] " << __FUNCTION__);\
} while (false)

#define CNR_RETURN_BOOL_THROTTLE_DEFAULT(logger, ret, ...)\
do\
{\
  double period = cnr_logger::getTraceLogger(logger) ? cnr_logger::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  if (ret) { CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__)); }\
  else     { CNR_ERROR_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));}\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, (ret ? "[  DONE] " : cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET())\
                                    << __FUNCTION__);\
} while (false);\
return ret;

#define CNR_RETURN_TRUE_THROTTLE_DEFAULT(logger,  ...)\
do\
{\
  double period = cnr_logger::getTraceLogger(logger) ? cnr_logger::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_RETURN_BOOL_THROTTLE(logger, true, period, __VA_ARGS__);\
} while(false);\


#define CNR_RETURN_FALSE_THROTTLE_DEFAULT(logger, ...)\
do\
{\
  double period = cnr_logger::getTraceLogger(logger) ? cnr_logger::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_RETURN_BOOL_THROTTLE(logger, false, period, __VA_ARGS__);\
} while(false);\

#define CNR_RETURN_OK_THROTTLE_DEFAULT(logger, var, ...)\
do\
{\
  double period = cnr_logger::getTraceLogger(logger) ? cnr_logger::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, "[  DONE] " << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_NOTOK_THROTTLE_DEFAULT(logger, var, ...)\
do\
{\
  double period = cnr_logger::getTraceLogger(logger) ? cnr_logger::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_ERROR_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET() << __FUNCTION__);\
} while (false);\
return(var);

#endif  // CNR_LOGGER_CNR_LOGGER_MACROS_H
