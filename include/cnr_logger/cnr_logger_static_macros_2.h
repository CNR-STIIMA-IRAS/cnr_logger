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
#ifndef CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_2_H
#define CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_2_H

#include <iostream>
#include <type_traits>
#include <cstdint>
#include <string>

#include <cnr_logger/cnr_logger_macros_support.h>

#define CNR_FATAL_2(__lgr, __msg) __log4cxx_helper(__lgr, cl::TraceLogger::Level::FATAL, __msg);

#define CNR_ERROR_2(__lgr, __msg) __log4cxx_helper(__lgr, cl::TraceLogger::Level::ERROR, __msg);

#define CNR_WARN_2(__lgr, __msg) __log4cxx_helper(__lgr, cl::TraceLogger::Level::WARN, __msg);

#define CNR_INFO_2(__lgr, __msg) __log4cxx_helper(__lgr, cl::TraceLogger::Level::INFO, __msg);

#define CNR_INFO_ONLY_FILE_2(__lgr, __msg)                                                                             \
  do                                                                                                                   \
  {                                                                                                                    \
    auto __l = cl::getTraceLogger(__lgr);                                                                              \
    if (__l && __l->logInfo())                                                                                         \
    {                                                                                                                  \
      if (__l->logSyncFileAndScreen())                                                                                 \
      {                                                                                                                \
        LOG4CXX_INFO(__l->syncFileAndScreenLogger(), __msg);                                                           \
      }                                                                                                                \
      if (__l->logOnlyFile())                                                                                          \
      {                                                                                                                \
        LOG4CXX_INFO(__l->fileLogger(), __msg);                                                                        \
      }                                                                                                                \
    }                                                                                                                  \
  } while (false);

#define CNR_DEBUG_2(__lgr, __msg) __log4cxx_helper(__lgr, cl::TraceLogger::Level::DEBUG, __msg);

#define CNR_TRACE_2(__lgr, __msg) __log4cxx_helper(__lgr, cl::TraceLogger::Level::TRACE, __msg);

#define CNR_FATAL_THROTTLE_2(__lgr, __period)                                                                          \
  CNR_FATAL_THROTTLE_3(__lgr, __period, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_ERROR_THROTTLE_2(__lgr, __period)                                                                          \
  CNR_ERROR_THROTTLE_3(__lgr, __period, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_WARN_THROTTLE_2(__lgr, __period)                                                                           \
  CNR_WARN_THROTTLE_3(__lgr, __period, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_INFO_THROTTLE_2(__lgr, __period)                                                                           \
  CNR_INFO_THROTTLE_2(__lgr, __period, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_DEBUG_THROTTLE_2(__lgr, __period)                                                                          \
  CNR_DEBUG_THROTTLE_3(__lgr, __period, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_TRACE_THROTTLE_2(__lgr, __period)                                                                          \
  CNR_TRACE_THROTTLE_3(__lgr, __period, cl::getTraceLogger(__lgr)->defaultMessage())

// =================

// ================= COND
#define CNR_FATAL_COND_2(__lgr, __cnd) CNR_FATAL_COND_3(__lgr, __cnd, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_ERROR_COND_2(__lgr, __cnd) CNR_FATAL_COND_3(__lgr, __cnd, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_WARN_COND_2(__lgr, __cnd) CNR_WARN_COND_3(__lgr, __cnd, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_INFO_COND_2(__lgr, __cnd) CNR_INFO_COND_3(__lgr, __cnd, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_DEBUG_COND_2(__lgr, __cnd) CNR_DEBUG_COND_3(__lgr, __cnd, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_TRACE_COND_2(__lgr, __cnd) CNR_TRACE_COND_3(__lgr, __cnd, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_RETURN_OK_2(__lgr, __ret_val) CNR_RETURN_OK_3(__lgr, __ret_val, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_RETURN_NOTOK_2(__lgr, __ret_val)                                                                           \
  CNR_RETURN_NOTOK_3(__lgr, __ret_val, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_RETURN_VOID_2(__lgr, __ret_ok, __msg)                                                                      \
  CNR_RETURN_VOID_3(__lgr, __ret_ok, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_RETURN_OK_THROTTLE_DEFAULT_2(__lgr, __ret_val)                                                             \
  CNR_RETURN_OK_THROTTLE_DEFAULT_3(__lgr, __ret_val, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_RETURN_NOTOK_THROTTLE_DEFAULT_2(__lgr, __ret_val)                                                          \
  CNR_RETURN_NOTOK_THROTTLE_DEFAULT_3(__lgr, __ret_val, cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_TRACE_START_2(__lgr, __msg)                                                                                \
  do                                                                                                                   \
  {                                                                                                                    \
    CNR_INFO_2(__lgr, __msg);                                                                                          \
    CNR_TRACE_2(__lgr, __fcn__start + std::string(__msg));                                                             \
  } while (false)

#define CNR_TRACE_END_2(__lgr, __msg)                                                                                  \
  do                                                                                                                   \
  {                                                                                                                    \
    CNR_INFO_2(__lgr, __msg);                                                                                          \
    CNR_TRACE_2(__lgr, __fcn__done + std::string(__msg));                                                              \
  } while (false)

#define CNR_RETURN_BOOL_2(__lgr, __ret_ok)                                                                             \
  CNR_RETURN_BOOL_3(__lgr, __ret_ok, , cl::getTraceLogger(__lgr)->defaultMessage())

#define CNR_RETURN_TRUE_2(__lgr, __msg)                                                                                \
  CNR_TRACE(__lgr, __fcn__done + cl::RESET() + __msg);                                                                 \
  return (true);

#define CNR_RETURN_FALSE_2(__lgr, __msg)                                                                               \
  CNR_TRACE(__lgr, __fcn__done + cl::RESET() + __msg);                                                                 \
  return (false);

#define CNR_RETURN_FATAL_2(__lgr, __msg)                                                                               \
  CNR_TRACE(__lgr, __fatal + __msg);                                                                                   \
  return (false);

#define CNR_EXIT_EX_2(__lgr, __ret_ok)                                                                                 \
  if (__ret_ok)                                                                                                        \
  {                                                                                                                    \
    __log4cxx_helper(__lgr, cl::TraceLogger::Level::TRACE, __fcn__done);                                               \
    return;                                                                                                            \
  }                                                                                                                    \
  else                                                                                                                 \
  {                                                                                                                    \
    __log4cxx_helper(__lgr, cl::TraceLogger::Level::TRACE, __failed);                                                  \
    throw std::invalid_argument(__failed.c_str());                                                                     \
  }

#define CNR_TRACE_START_THROTTLE_2(__lgr, __period) CNR_TRACE_START_THROTTLE_3(__lgr, __period, __fcn__start)

#define CNR_RETURN_TRUE_THROTTLE_2(__lgr, __period)                                                                    \
  CNR_TRACE_THROTTLE_3(__lgr, __period, __true);                                                                       \
  return (true)

#define CNR_RETURN_FALSE_THROTTLE_2(__lgr, __period)                                                                   \
  CNR_TRACE_THROTTLE_3(__lgr, __period, __false);                                                                      \
  return (false)

#define CNR_TRACE_START_THROTTLE_DEFAULT_2(__lgr, __msg)                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    auto __period = cl::getTraceLogger(__lgr)->defaultThrottleTime();                                                  \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, __msg);                                  \
  } while (false)

#define CNR_RETURN_BOOL_THROTTLE_DEFAULT_2(__lgr, __ret_ok)                                                            \
  do                                                                                                                   \
  {                                                                                                                    \
    auto __period = cl::getTraceLogger(__lgr)->defaultThrottleTime();                                                  \
    if (__ret_ok)                                                                                                      \
    {                                                                                                                  \
      __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, __true);                               \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
      __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, __false);                              \
    }                                                                                                                  \
  } while (false);                                                                                                     \
  return (__ret_ok)

#define CNR_RETURN_TRUE_THROTTLE_DEFAULT_2(__lgr, __msg)                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    auto __period = cl::getTraceLogger(__lgr)->defaultThrottleTime();                                                  \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::INFO, __period, __msg);                                   \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, __true);                                 \
  } while (false);                                                                                                     \
  return (true)

#define CNR_RETURN_FALSE_THROTTLE_DEFAULT_2(__lgr, __msg)                                                              \
  do                                                                                                                   \
  {                                                                                                                    \
    auto __period = cl::getTraceLogger(__lgr)->defaultThrottleTime();                                                  \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::INFO, __period, __msg);                                   \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, __false);                                \
  } while (false);                                                                                                     \
  return (false)

#endif  // CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_2_H
