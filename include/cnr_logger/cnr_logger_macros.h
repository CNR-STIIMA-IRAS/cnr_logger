/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2020 National Council of Research of Italy (CNR)
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
 * @author Nicola Pedrocchi
 * @date 25 Jun 2020
 * @brief File containing the macro definition.
 *
 * The macro have been designed to follow the basic log4cxx structure (and ROS).
 * The further macro CNR_TRACE_START and CNR_RETURN_xxx are used to trace the input and the output values of the functions
 */

#ifndef CNR_LOGGER_CNR_LOGGER_MACROS_H
#define CNR_LOGGER_CNR_LOGGER_MACROS_H

#include <string>
#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <log4cxx/rollingfileappender.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>
#include <ros/console.h>

namespace log4cxx
{
class LOG4CXX_EXPORT ColorPatternLayout : public log4cxx::PatternLayout
{
public:
  DECLARE_LOG4CXX_OBJECT(ColorPatternLayout)
  BEGIN_LOG4CXX_CAST_MAP()
  LOG4CXX_CAST_ENTRY(ColorPatternLayout)
  LOG4CXX_CAST_ENTRY_CHAIN(Layout)
  END_LOG4CXX_CAST_MAP()

  ColorPatternLayout() : log4cxx::PatternLayout() {}
  explicit ColorPatternLayout(const log4cxx::LogString &s)  
  : log4cxx::PatternLayout(s) 
  {

  }
  virtual void format(log4cxx::LogString &output,
                      const log4cxx::spi::LoggingEventPtr &event,
                      log4cxx::helpers::Pool &pool) const override
  {
    log4cxx::LogString tmp;
    log4cxx::PatternLayout::format(tmp, event, pool);
    log4cxx::LevelPtr lvl = event->getLevel();
    switch (lvl->toInt())
    {
    case log4cxx::Level::FATAL_INT:
      output.append("\u001b[0;41m");  // red BG
      break;
    case log4cxx::Level::ERROR_INT:
      output.append("\u001b[0;31m");  // red FG
      break;
    case log4cxx::Level::WARN_INT:
      output.append("\u001b[0;33m");  // Yellow FG
      break;
    case log4cxx::Level::INFO_INT:
      output.append("\u001b[1m");     // Bright
      break;
    case log4cxx::Level::DEBUG_INT:
      output.append("\u001b[1;32m");  // Green FG
      break;
    case log4cxx::Level::TRACE_INT:
      output.append("\u001b[0;34m");  // Black FG
      break;
    default:
      break;
    }
    output.append(tmp);
    output.append("\u001b[m");
  }
};
LOG4CXX_PTR_DEF(ColorPatternLayout);

}  // namespace log4cxx


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
  return GREE();
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


}  // namespace cnr_logger


// ================= STANDARD
#define CNR_FATAL(trace_logger, args)\
do \
{ \
  if ((&(trace_logger))->logSyncFileAndScreen())\
    LOG4CXX_FATAL((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
  else if ((&(trace_logger))->logFile())\
      LOG4CXX_FATAL((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
  else if ((&(trace_logger))->logScreen())\
      LOG4CXX_FATAL((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
} while (false)

#define CNR_ERROR(trace_logger, args)\
do \
{\
  if ((&(trace_logger))->logSyncFileAndScreen())\
    LOG4CXX_ERROR((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
  else if ((&(trace_logger))->logFile())\
      LOG4CXX_ERROR((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
  else if ((&(trace_logger))->logScreen())\
      LOG4CXX_ERROR((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
} while (false)

#define CNR_WARN(trace_logger, args)\
do\
{\
  if ((&(trace_logger))->logSyncFileAndScreen())\
    LOG4CXX_WARN((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
  else if ((&(trace_logger))->logFile())\
      LOG4CXX_WARN((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
  else if ((&(trace_logger))->logScreen())\
      LOG4CXX_WARN((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
} while (false)

#define CNR_INFO(trace_logger, args)\
do\
{\
  if (((&(trace_logger))->logSyncFileAndScreen())) \
    LOG4CXX_INFO((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
  else if ((&(trace_logger))->logFile()) \
    LOG4CXX_INFO((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
  else if ((&(trace_logger))->logScreen()) \
      LOG4CXX_INFO((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
} while (false)

#define CNR_INFO_ONLY_FILE(trace_logger, args)\
do\
{\
  if (((&(trace_logger))->logSyncFileAndScreen())) \
    LOG4CXX_INFO((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
  else if ((&(trace_logger))->logFile()) \
    LOG4CXX_INFO((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
} while (false)

#define CNR_DEBUG(trace_logger, args)\
do\
{\
  if ((&(trace_logger))->logSyncFileAndScreen())\
    LOG4CXX_DEBUG((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
  else if ((&(trace_logger))->logFile())\
    LOG4CXX_DEBUG((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
  else if ((&(trace_logger))->logScreen())\
    LOG4CXX_DEBUG((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
} while (false)

#define CNR_TRACE(trace_logger, args)\
do\
{\
  if ((&(trace_logger))->logSyncFileAndScreen())\
    LOG4CXX_TRACE((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::SYNC_FILE_AND_CONSOLE], args)\
  else if ((&(trace_logger))->logFile())\
    LOG4CXX_TRACE((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::FILE_STREAM], args)\
  else if ((&(trace_logger))->logScreen())\
    LOG4CXX_TRACE((&(trace_logger))->loggers_[::cnr_logger::TraceLogger::CONSOLE_STREAM], args)\
} while (false)
// =================


// ================= THROTTLE
#define CNR_FATAL_THROTTLE(trace_logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = ::ros::Time::now().toSec(); \
    if (ROSCONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_FATAL(trace_logger, args); \
    } \
  } while (false)

#define CNR_ERROR_THROTTLE(trace_logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = ::ros::Time::now().toSec(); \
    if (ROSCONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_ERROR(trace_logger, args); \
    } \
  } while (false)

#define CNR_WARN_THROTTLE(trace_logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = ::ros::Time::now().toSec(); \
    if (ROSCONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_WARN(trace_logger, args); \
    } \
  } while (false)

#define CNR_INFO_THROTTLE(trace_logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = ::ros::Time::now().toSec(); \
    if (ROSCONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_INFO(trace_logger, args); \
    } \
  } while (false)

#define CNR_DEBUG_THROTTLE(trace_logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = ::ros::Time::now().toSec(); \
    if (ROSCONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_DEBUG(trace_logger, args); \
    } \
  } while (false)


#define CNR_TRACE_THROTTLE(trace_logger, period, args)\
  do \
  { \
    static double __log_throttle_last_hit__ = 0.0; \
    double __log_throttle_now__ = ::ros::Time::now().toSec(); \
    if (ROSCONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, period))\
    { \
      __log_throttle_last_hit__ = __log_throttle_now__; \
      CNR_TRACE(trace_logger, args); \
    } \
  } while (false)
// =================


// ================= COND
#define CNR_FATAL_COND(trace_logger, cond, args)\
  do\
  {\
    if (cond) { CNR_FATAL(trace_logger, args); }\
  } while (false)

#define CNR_ERROR_COND(trace_logger, cond, args)\
  do\
  {\
    if (cond) { CNR_ERROR(trace_logger, args); }\
  } while (false)

#define CNR_WARN_COND(trace_logger, cond, args)\
  do\
  {\
    if (cond) { CNR_WARN(trace_logger, args); }\
} while (false)

#define CNR_INFO_COND(trace_logger, cond, args)\
  do\
  {\
    if (cond) { CNR_INFO(trace_logger, args); }\
  } while (false)

#define CNR_DEBUG_COND(trace_logger, cond, args)\
  do\
  {\
    if (cond) { CNR_DEBUG(trace_logger, args); }\
  } while (false)

#define CNR_TRACE_COND(trace_logger, cond, args)\
  do\
  {\
    if (cond) { CNR_TRACE(trace_logger, args); }\
  } while (false)


// ================= COND THROTTLE
#define CNR_FATAL_COND_THROTTLE(trace_logger, cond, period, args)\
do\
{\
  if (cond) { CNR_FATAL_THROTTLE(trace_logger, period, args); }\
} while (false)

#define CNR_ERROR_COND_THROTTLE(trace_logger, cond, period, args)\
do\
{\
  if (cond) { CNR_ERROR_THROTTLE(trace_logger, period, args); }\
} while (false)

#define CNR_WARN_COND_THROTTLE(trace_logger, cond, period, args)\
do\
{\
  if (cond) { CNR_WARN_THROTTLE(trace_logger, period, args); }\
} while (false)

#define CNR_INFO_COND_THROTTLE(trace_logger, cond, period, args)\
do\
{\
  if (cond) { CNR_INFO_THROTTLE(trace_logger, period, args); }\
} while (false)

#define CNR_DEBUG_COND_THROTTLE(trace_logger, cond, period, args)\
do\
{\
  if (cond) { CNR_DEBUG_THROTTLE(trace_logger, period, args); }\
} while (false)

#define CNR_TRACE_COND_THROTTLE(trace_logger, cond, period, args)\
do\
{\
  if (cond) { CNR_TRACE_THROTTLE(trace_logger, period, args); }\
} while (false)




// ============================== IN/OUT Functions
#define CNR_TRACE_START(logger, ...)\
do\
{\
  CNR_INFO_COND(logger, (std::string(__VA_ARGS__).length() > 0), std::string(__VA_ARGS__)); \
  CNR_TRACE(logger, "[ START] " << __FUNCTION__);\
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
  if (ret)   { CNR_INFO_COND(logger, std::string(__VA_ARGS__).length() > 0, std::string(__VA_ARGS__));  }\
  else       { CNR_ERROR_COND(logger, std::string(__VA_ARGS__).length() > 0, std::string(__VA_ARGS__));  }\
  CNR_TRACE(logger, (ret ? "[  DONE] " : cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET()) << __FUNCTION__);\
} while (false);\
return ret;\


#define CNR_RETURN_TRUE(logger,  ...)\
  CNR_RETURN_BOOL(logger, true, __VA_ARGS__);

#define CNR_RETURN_FALSE(logger, ...)\
  CNR_RETURN_BOOL(logger, false, __VA_ARGS__);

#define CNR_RETURN_FATAL(logger, ...)\
  do\
  {\
    CNR_FATAL_COND(logger, std::string(__VA_ARGS__).length() > 0, __VA_ARGS__);\
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


//
#define CNR_TRACE_START_THROTTLE(logger, period, ...)\
do\
{\
  CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__)); \
  CNR_TRACE_THROTTLE(logger, period, "[ START] " << __FUNCTION__);\
} while (false)

#define CNR_RETURN_BOOL_THROTTLE(logger, ret, period, ...)\
do\
{\
  if (ret) { CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__)); }\
  else     { CNR_ERROR_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));}\
  CNR_TRACE_THROTTLE(logger, period, (ret ? "[  DONE] " : cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET())\
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
  CNR_TRACE_THROTTLE(logger, period, "[  DONE] " << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_NOTOK_THROTTLE(logger, var, period, ...)\
do\
{\
  CNR_ERROR_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));\
  CNR_TRACE_THROTTLE(logger, period, cnr_logger::RED() + "[FAILED] " + cnr_logger::RESET() << __FUNCTION__);\
} while (false);\
return(var);


#endif  // CNR_LOGGER_CNR_LOGGER_MACROS_H
