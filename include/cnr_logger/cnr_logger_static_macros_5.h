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
#ifndef CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_5_H
#define CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_5_H

#include <cnr_logger/cnr_logger_macros_support.h>

#define CNR_FATAL_5(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_5(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_5(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_5(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_5(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_5(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_6(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_6(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_6(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_6(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_6(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_6(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_7(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_7(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_7(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_7(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_7(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_7(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_8(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_8(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_8(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_8(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_8(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_8(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_9(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_9(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_9(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_9(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_9(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_9(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_10(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_10(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_10(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_10(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_10(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_10(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_11(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_11(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_11(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_11(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_11(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_11(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_12(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_12(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_12(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_12(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_12(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_12(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_13(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_13(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_13(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_13(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_13(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_13(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_14(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_14(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_14(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_14(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_14(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_14(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_15(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_15(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_15(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_15(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_15(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_15(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_16(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_16(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_16(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_16(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_16(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_16(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_17(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_17(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_17(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_17(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_17(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_17(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_18(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_18(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_18(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_18(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_18(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_18(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#define CNR_FATAL_19(__lgr, __format, ...)  CNR_FATAL_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_ERROR_19(__lgr, __format, ...)  CNR_ERROR_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_WARN_19(__lgr, __format, ...)  CNR_WARN_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_INFO_19(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_DEBUG_19(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))
#define CNR_TRACE_19(__lgr, __format, ...)  CNR_INFO_2(__lgr, make_string(__format, __VA_ARGS__))

#endif  // CNR_LOGGER_CNR_LOGGER_STATIC_MACROS_5_H
