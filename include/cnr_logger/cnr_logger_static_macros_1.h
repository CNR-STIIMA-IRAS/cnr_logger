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
#ifndef CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_1_H
#define CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_1_H

#include <iostream>
#include <type_traits>
#include <cstdint>
#include <string>

#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_static_macros_2.h>

// ================= STANDARD
#define CNR_FATAL_1(___lgr) CNR_FATAL_2(___lgr, __fatal)

#define CNR_ERROR_1(___lgr) CNR_ERROR_2(___lgr, __error)

#define CNR_WARN_1(___lgr) CNR_WARN_2(___lgr, __warn)

#define CNR_INFO_1(___lgr) CNR_INFO_2(___lgr, cl::getTraceLogger(___lgr)->defaultMessage())

#define CNR_INFO_ONLY_FILE_1(___lgr) CNR_INFO_ONLY_FILE_2(___lgr, cl::getTraceLogger(___lgr)->defaultMessage())

#define CNR_DEBUG_1(___lgr) CNR_DEBUG_2(___lgr, cl::getTraceLogger(___lgr)->defaultMessage())

#define CNR_TRACE_1(___lgr) CNR_TRACE_2(___lgr, cl::getTraceLogger(___lgr)->defaultMessage())

#define CNR_TRACE_START_1(___lgr) CNR_TRACE_2(___lgr, __fcn__start)

#define CNR_TRACE_END_1(___lgr) CNR_TRACE_2(___lgr, __fcn__done)

#define CNR_RETURN_TRUE_1(___lgr)                                                                                      \
  CNR_TRACE_2(___lgr, __fcn__done);                                                                                    \
  return (true);

#define CNR_RETURN_FALSE_1(___lgr)                                                                                     \
  CNR_TRACE_2(___lgr, __fcn__done);                                                                                    \
  return (false);

#define CNR_RETURN_FATAL_1(___lgr)                                                                                     \
  CNR_TRACE_2(___lgr, __failed);                                                                                       \
  return (false);

#define CNR_EXIT_EX_1(___lgr, ok)                                                                                      \
  if (ok)                                                                                                              \
  {                                                                                                                    \
    CNR_TRACE_2(___lgr, __fcn__done);                                                                                  \
  }                                                                                                                    \
  else                                                                                                                 \
  {                                                                                                                    \
    CNR_TRACE_2(___lgr, __failed);                                                                                     \
    throw std::invalid_argument(__failed.c_str());                                                                     \
  }                                                                                                                    \
  return;

#define CNR_TRACE_START_THROTTLE_DEFAULT_1(___lgr)                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    __log4cxx_helper_throttle(___lgr, cl::TraceLogger::Level::TRACE,                                                   \
                              (cl::getTraceLogger(___lgr)->defaultThrottleTime()), __fcn__start);                      \
  } while (false)

#define CNR_RETURN_TRUE_THROTTLE_DEFAULT_1(___lgr)                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    auto period = cl::getTraceLogger(___lgr)->defaultThrottleTime();                                                   \
    __log4cxx_helper_throttle(___lgr, cl::TraceLogger::Level::TRACE, period, __true);                                  \
  } while (false);                                                                                                     \
  return (true)

#define CNR_RETURN_FALSE_THROTTLE_DEFAULT_1(___lgr)                                                                    \
  do                                                                                                                   \
  {                                                                                                                    \
    auto period = cl::getTraceLogger(___lgr)->defaultThrottleTime();                                                   \
    __log4cxx_helper_throttle(___lgr, cl::TraceLogger::Level::TRACE, period, __false);                                 \
  } while (false);                                                                                                     \
  return (false)

#endif  // CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_1_H
