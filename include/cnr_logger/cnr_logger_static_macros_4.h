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
#ifndef CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_4_H
#define CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_4_H

#include <cnr_logger/cnr_logger_macros_support.h>

#define CNR_FATAL_4(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_4(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_4(__lgr, __format, ...)   CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_4(__lgr, __format, ...)   CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_4(__lgr, __format, ...)  CNR_DEBUG_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_4(__lgr, __format, ...)  CNR_TRACE_2(__lgr, make_string(__format, __VA_ARGS__))

// ================= COND THROTTLE
#define CNR_FATAL_COND_THROTTLE_4(__lgr, __cnd, __period, __msg)                                                       \
  if (__cnd)                                                                                                           \
  {                                                                                                                    \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::FATAL, __period, __msg);                                  \
  }

#define CNR_ERROR_COND_THROTTLE_4(__lgr, __cnd, __period, __msg)                                                       \
  if (__cnd)                                                                                                           \
  {                                                                                                                    \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::ERROR, __period, __msg);                                  \
  }

#define CNR_WARN_COND_THROTTLE_4(__lgr, __cnd, __period, __msg)                                                        \
  if (__cnd)                                                                                                           \
  {                                                                                                                    \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::WARN, __period, __msg);                                   \
  }

#define CNR_INFO_COND_THROTTLE_4(__lgr, __cnd, __period, __msg)                                                        \
  if (__cnd)                                                                                                           \
  {                                                                                                                    \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::INFO, __period, __msg);                                   \
  }

#define CNR_DEBUG_COND_THROTTLE_4(__lgr, __cnd, __period, __msg)                                                       \
  if (__cnd)                                                                                                           \
  {                                                                                                                    \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::DEBUG, __period, __msg);                                  \
  }

#define CNR_TRACE_COND_THROTTLE_4(__lgr, __cnd, __period, __msg)                                                       \
  if (__cnd)                                                                                                           \
  {                                                                                                                    \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, __msg);                                  \
  }

#define CNR_RETURN_OK_THROTTLE_4(__lgr, var, __period, __msg)                                                          \
  do                                                                                                                   \
  {                                                                                                                    \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::INFO, __period, __msg);                                   \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, __fcn__done);                            \
  } while (false);                                                                                                     \
  return (var);

#define CNR_RETURN_NOTOK_THROTTLE_4(__lgr, var, __period, __msg)                                                       \
  do                                                                                                                   \
  {                                                                                                                    \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::ERROR, __period, __msg);                                  \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, __failed);                               \
  } while (false);                                                                                                     \
  return (var);

#define CNR_RETURN_BOOL_THROTTLE_4(__lgr, __ret_ok, __period, __msg)                                                   \
  do                                                                                                                   \
  {                                                                                                                    \
    if (__ret_ok)                                                                                                      \
    {                                                                                                                  \
      __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::INFO, __period, __msg);                                 \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
      __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::ERROR, __period, __msg);                                \
    }                                                                                                                  \
    __log4cxx_helper_throttle(__lgr, cl::TraceLogger::Level::TRACE, __period, (__ret_ok ? __fcn__done : __failed));    \
  } while (false);                                                                                                     \
  return (__ret_ok)

#endif  // CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_4_H
