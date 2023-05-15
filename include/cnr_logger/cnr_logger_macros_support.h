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
 * The further macro CNR_TRACE_START and CNR_RETURN_xxx are used to trace the input and the output values of the
 * functions
 */
#ifndef CNR_LOGGER_CNR_LOGGER_MACROS_SUPPORT__H
#define CNR_LOGGER_CNR_LOGGER_MACROS_SUPPORT__H

#include <iostream>
#include <type_traits>
#include <cstdint>
#include <string>
#include <sstream>

#include <cnr_logger/cnr_logger.h>

// user can force the use of ROS time by adding "-DFORCE_ROS_TIME_USE"

#if defined(FORCE_ROS_TIME_USE)
#include <ros/time.h>
#include <ros/console.h>
#define CONSOLE_THROTTLE_CHECK(now, last, period) ROSCONSOLE_THROTTLE_CHECK(now, last, period)

#define TIME_NOW() ::ros::Time::now().toSec()
#else

#include <ctime>
#if defined(_MSC_VER)
#define CNR_LIKELY(x) (x)
#define CNR_UNLIKELY(x) (x)
#else
#define CNR_LIKELY(x) __builtin_expect((x), 1)
#define CNR_UNLIKELY(x) __builtin_expect((x), 0)
#endif

#define CONSOLE_THROTTLE_CHECK(now, last, period) (CNR_UNLIKELY(last + period <= now) || CNR_UNLIKELY(now < last))

#define TIME_NOW() std::time(0)
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
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": The logger is not initialized. Fake log enabled."
              << std::endl;
    auto ret = std::make_shared<TraceLogger>();
    return ret.get();
  }
}
}  // namespace cnr_logger

//! Shortcut
namespace cl = cnr_logger;

#define __log4cxx_helper(___lgr, level, ...)                                                                           \
  do                                                                                                                   \
  {                                                                                                                    \
    std::stringstream ss;                                                                                              \
    ss << __VA_ARGS__;                                                                                                 \
    auto __l = cl::getTraceLogger(___lgr);                                                                             \
    if (__l && (ss.str().length() > 0))                                                                                \
    {                                                                                                                  \
      if (level == cl::TraceLogger::Level::FATAL && __l->logFatal())                                                   \
      {                                                                                                                \
        if (__l->logSyncFileAndScreen())                                                                               \
          LOG4CXX_FATAL(__l->syncFileAndScreenLogger(), ss.str());                                                     \
        if (__l->logOnlyFile())                                                                                        \
          LOG4CXX_FATAL(__l->fileLogger(), ss.str());                                                                  \
        if (__l->logOnlyScreen())                                                                                      \
          LOG4CXX_FATAL(__l->consoleLogger(), ss.str());                                                               \
      }                                                                                                                \
      else if (level == cl::TraceLogger::Level::ERROR && __l->logError())                                              \
      {                                                                                                                \
        if (__l->logSyncFileAndScreen())                                                                               \
          LOG4CXX_ERROR(__l->syncFileAndScreenLogger(), ss.str());                                                     \
        if (__l->logOnlyFile())                                                                                        \
          LOG4CXX_ERROR(__l->fileLogger(), ss.str());                                                                  \
        if (__l->logOnlyScreen())                                                                                      \
          LOG4CXX_ERROR(__l->consoleLogger(), ss.str());                                                               \
      }                                                                                                                \
      else if (level == cl::TraceLogger::Level::WARN && __l->logWarn())                                                \
      {                                                                                                                \
        if (__l->logSyncFileAndScreen())                                                                               \
          LOG4CXX_WARN(__l->syncFileAndScreenLogger(), ss.str());                                                      \
        if (__l->logOnlyFile())                                                                                        \
          LOG4CXX_WARN(__l->fileLogger(), ss.str());                                                                   \
        if (__l->logOnlyScreen())                                                                                      \
          LOG4CXX_WARN(__l->consoleLogger(), ss.str());                                                                \
      }                                                                                                                \
      else if (level == cl::TraceLogger::Level::INFO && __l->logInfo())                                                \
      {                                                                                                                \
        if (__l->logSyncFileAndScreen())                                                                               \
          LOG4CXX_INFO(__l->syncFileAndScreenLogger(), ss.str());                                                      \
        if (__l->logOnlyFile())                                                                                        \
          LOG4CXX_INFO(__l->fileLogger(), ss.str());                                                                   \
        if (__l->logOnlyScreen())                                                                                      \
          LOG4CXX_INFO(__l->consoleLogger(), ss.str());                                                                \
      }                                                                                                                \
      else if (level == cl::TraceLogger::Level::DEBUG && __l->logDebug())                                              \
      {                                                                                                                \
        if (__l->logSyncFileAndScreen())                                                                               \
          LOG4CXX_DEBUG(__l->syncFileAndScreenLogger(), ss.str());                                                     \
        if (__l->logOnlyFile())                                                                                        \
          LOG4CXX_DEBUG(__l->fileLogger(), ss.str());                                                                  \
        if (__l->logOnlyScreen())                                                                                      \
          LOG4CXX_DEBUG(__l->consoleLogger(), ss.str());                                                               \
      }                                                                                                                \
      else if (level == cl::TraceLogger::Level::TRACE && __l->logTrace())                                              \
      {                                                                                                                \
        if (__l->logSyncFileAndScreen())                                                                               \
          LOG4CXX_TRACE(__l->syncFileAndScreenLogger(), ss.str());                                                     \
        if (__l->logOnlyFile())                                                                                        \
          LOG4CXX_TRACE(__l->fileLogger(), ss.str());                                                                  \
        if (__l->logOnlyScreen())                                                                                      \
          LOG4CXX_TRACE(__l->consoleLogger(), ss.str());                                                               \
      }                                                                                                                \
    }                                                                                                                  \
  } while (false)

#define __log4cxx_helper_throttle(___lgr, ___lvl, __period, ...)                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    if (__period <= 0)                                                                                                 \
    {                                                                                                                  \
      std::cerr << __PRETTY_FUNCTION__ << ": " << __LINE__ << ": Error in the MACRO. Period is negative " << std::endl; \
      exit(-1);                                                                                                        \
    }                                                                                                                  \
    static double __log_throttle_last_hit__ = 0.0;                                                                     \
    double __log_throttle_now__ = TIME_NOW();                                                                          \
    if (CONSOLE_THROTTLE_CHECK(__log_throttle_now__, __log_throttle_last_hit__, __period))                             \
    {                                                                                                                  \
      __log_throttle_last_hit__ = __log_throttle_now__;                                                                \
      __log4cxx_helper(___lgr, ___lvl, __VA_ARGS__);                                                                   \
    }                                                                                                                  \
  } while (false)

#define __fcn__start std::string(cl::BOLDGREEN() + "[ START] " + cl::RESET() + __PRETTY_FUNCTION__)

#define __fcn__done std::string(cl::BOLDYELLOW() + "[  DONE] " + cl::RESET() + __PRETTY_FUNCTION__)

#define __failed std::string(cl::RED() + "[FAILED] " + cl::RESET() + __PRETTY_FUNCTION__)

#define __fatal std::string(cl::RED() + "[ FATAL] " + cl::RESET() + __PRETTY_FUNCTION__)

#define __error std::string(cl::RED() + "[ ERROR] " + cl::RESET() + __PRETTY_FUNCTION__)

#define __warn std::string(cl::BOLDYELLOW() + "[  WARN] " + cl::RESET() + __PRETTY_FUNCTION__)

#define __false std::string(cl::BOLDMAGENTA() + "[ FALSE] " + cl::RESET() + __PRETTY_FUNCTION__)

#define __true std::string(cl::BOLDMAGENTA() + "[  TRUE] " + cl::RESET() + __PRETTY_FUNCTION__)

// ============================== VARIADIC MACRO SUPPORT
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

  void pp_get()
  {
  }
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
    msg_ += (msg_.empty() ? first : (", " + first));
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

#endif  // CNR_LOGGER_CNR_LOGGER_MACROS_SUPPORT__H
