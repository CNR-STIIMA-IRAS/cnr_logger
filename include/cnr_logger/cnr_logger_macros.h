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
#ifndef CNR_LOGGER_CNR_LOGGER_MACROS_H
#define CNR_LOGGER_CNR_LOGGER_MACROS_H

#include <iostream>
#include <type_traits>
#include <cstdint>
#include <string>

#if defined(_MSC_VER)
  #define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif


#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_color_macros.h>
#include <cnr_logger/cnr_logger_static_macros.h>
#include <cnr_logger/cnr_logger_variadic_macros.h>



<<<<<<< HEAD
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
    auto ret = std::make_shared<TraceLogger>();
    return ret.get();
  }
}




}  // namespace cnr_logger



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

// ============================== IN/OUT Functions



struct VariadicParser
{
  cl::TraceLogger* logger_ = nullptr;
  std::string msg_ = "";
  bool ok_ = true;
  double period_ = -1.0;

  VariadicParser() = default;

  template <class First, class... Rest>
  VariadicParser(First first, Rest... rest)
  {
    pp_get(first, rest...);
  }
  template <class... Rest>
  VariadicParser(cl::TraceLogger& first, Rest... rest)
  {
    pp_get(first, rest...);
  }
  template <class... Rest> 
  VariadicParser(cl::TraceLogger* first, Rest... rest)
  {
    pp_get(first, rest...);
  }
  template <class... Rest> 
  VariadicParser(std::shared_ptr<cl::TraceLogger>& first, Rest... rest)
  {
    pp_get(first, rest...);
  }
  template <class... Rest> 
  VariadicParser(const std::string& first, Rest... rest)
  {
    pp_get(first, rest...);
  }
  template <class... Rest> 
  VariadicParser(const char* first, Rest... rest)
  {
    pp_get(first, rest...);
  }
  template <class... Rest> 
  VariadicParser(const double& first, Rest... rest)
  {
    pp_get(first, rest...);
  }
  template <class... Rest> 
  VariadicParser(const int& first, Rest... rest)
  {
    pp_get(first, rest...);
  }


  void pp_get() { }
  template <class First, class... Rest>
  void pp_get(First first, Rest... rest)
  {
    pp_get(first, rest...);
  }
  template <class... Rest>
  void pp_get(cl::TraceLogger& first, Rest... rest)
  {
    logger_ = cl::getTraceLogger(first);
    pp_get(rest...);
  }
  template <class... Rest>
  void pp_get(cl::TraceLogger* first, Rest... rest)
  {
    logger_ = cl::getTraceLogger(first);
    pp_get(rest...);
  }
  template <class... Rest>
  void pp_get(std::shared_ptr<cl::TraceLogger>& first, Rest... rest)
  {
    logger_ = cl::getTraceLogger(first);
    pp_get(rest...);
  }
  template <class... Rest>
  void pp_get(const std::string& first, Rest... rest)
  {
    msg_ += (msg_.empty() ? first : (", " + first) );
    pp_get(rest...);
  }
  template <class... Rest>
  void pp_get(const char* first, Rest... rest)
  {
    msg_ += (msg_.empty() ? first : std::string(", ") + first);
    pp_get(rest...);
  }
  template <class... Rest>
  void pp_get(const double& first, Rest... rest)
  {
    period_ = first;
    pp_get(rest...);
  }
  template <class... Rest>
  void pp_get(const int& first, Rest... rest)
  {
    ok_ = static_cast<bool>(first);
    pp_get(rest...);
  }
  void default_period()
  {
    period_ = logger_ ? logger_->defaultThrottleTime() : 1.0;
  }
};

#define CNR_TRACE_START(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  CNR_TRACE(e.logger_, \
    cl::YELLOW() + "[ START] " + cl::RESET() << ( e.msg_.empty() ? __FUNCTION__ : e.msg_) );\
} while (false)

#define CNR_TRACE_END(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  CNR_INFO_COND(e.logger_, !e.msg_.empty(), e.msg_); \
  CNR_TRACE(e.logger_, "[  DONE] " << __FUNCTION__);\
} while (false)


#define CNR_RETURN_BOOL(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  std::string result = (e.ok_ ? cl::GREEN()+"[  DONE] " : cl::RED()+"[FAILED] ") + cl::RESET();\
  CNR_TRACE(e.logger_, result << (e.msg_.empty() ? __FUNCTION__ : e.msg_ ) );\
  return(e.ok_);\
} while (false);\


#define CNR_RETURN_TRUE(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  \
  std::string msg = cl::GREEN()+"[  DONE] " + cl::RESET() + (e.msg_.empty() ? __FUNCTION__ : e.msg_ );\
  CNR_TRACE(e.logger_, msg);\
} while (false);\
return(true);

#define CNR_RETURN_FALSE(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  \
  std::string msg = cl::RED()+"[FAILED] " + cl::RESET() + (e.msg_.empty() ? __FUNCTION__ : e.msg_ );\
  CNR_TRACE(e.logger_, msg );\
} while (false);\
return(false);

#define CNR_RETURN_FATAL(...)\
  do\
  {\
    VariadicParser e(__VA_ARGS__);\
    \
    CNR_FATAL_COND(e.logger_, e.msg_.length(), e.msg_);\
    CNR_TRACE(e.logger_, (cl::RED() + "[FAILED] " + cl::RESET())<< __FUNCTION__);\
  } while (false);\
  return false;\


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

#define CNR_EXIT_EX(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  if(e.ok_) \
  {\
    CNR_ERROR_COND(e.logger_, e.msg_.length() > 0, e.msg_); \
    CNR_TRACE(e.logger_, "[  DONE] " << __FUNCTION__);\
    return;\
  }\
  else\
  {\
    CNR_INFO_COND(e.logger_, e.msg_.length() > 0, e.msg_);\
    CNR_TRACE(e.logger_, cl::RED() + "[FAILED] " + cl::RESET() << __FUNCTION__);\
    throw std::invalid_argument(e.msg_.c_str());\
  }\
} while (false)


// ============================== IN/OUT Functions THROTTLE
#define CNR_TRACE_START_THROTTLE(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_); \
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, "[ START] " << __FUNCTION__);\
} while (false)

#define CNR_RETURN_BOOL_THROTTLE(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  if (e.ok_){ CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_); }\
  else      { CNR_ERROR_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_);}\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, \
      (e.ok_ ? "[  DONE] " : cl::RED() + "[FAILED] " + cl::RESET()) << __FUNCTION__);\
  return(e.ok_);\
} while (false);\


#define CNR_RETURN_TRUE_THROTTLE(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_);\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, ("[  DONE] "+cl::RESET()) << __FUNCTION__);\
} while (false);\
return(true);

#define CNR_RETURN_FALSE_THROTTLE(...)\
do\
{\
  VariadicParser e; e.pp_get(__VA_ARGS__);\
  CNR_ERROR_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_);\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, ("[  DONE] "+cl::RESET()) << __FUNCTION__);\
} while (false);\
return(false);

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




// ============================== IN/OUT Functions THROTTLE DEFAULT
#define CNR_TRACE_START_THROTTLE_DEFAULT(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  e.default_period();\
  CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length() > 0, e.period_, e.msg_); \
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, "[ START] " << __FUNCTION__);\
} while (false)

#define CNR_RETURN_BOOL_THROTTLE_DEFAULT(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  e.default_period();\
  if (e.ok_){ CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length() > 0, e.period_, e.msg_); }\
  else      { CNR_ERROR_COND_THROTTLE(e.logger_, e.msg_.length() > 0, e.period_, e.msg_);}\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_,\
      (e.ok_ ? "[  DONE] " : cl::RED() + "[FAILED] " + cl::RESET())  << __FUNCTION__);\
  return(e.ok_);\
} while (false);

#define CNR_RETURN_TRUE_THROTTLE_DEFAULT(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  e.default_period();\
  CNR_INFO_COND_THROTTLE( e.logger_, e.msg_.length() > 0, e.period_, e.msg_);\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, "[  DONE] " << __FUNCTION__);\
  return(true);\
} while(false);\


#define CNR_RETURN_FALSE_THROTTLE_DEFAULT(...)\
do\
{\
  VariadicParser e(__VA_ARGS__);\
  e.default_period();\
  CNR_ERROR_COND_THROTTLE(e.logger_, e.msg_.length() > 0, e.period_, e.msg_);\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, "[  DONE] " << __FUNCTION__);\
  return(false);\
} while(false);\

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
=======
>>>>>>> b5d9ad8d7a55b45cc01a0780856bc6010bc458ae

#endif  // CNR_LOGGER_CNR_LOGGER_MACROS_H
