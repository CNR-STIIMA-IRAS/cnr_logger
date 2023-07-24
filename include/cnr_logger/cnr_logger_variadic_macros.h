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
#ifndef CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_H
#define CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_H

#include <iostream>
#include <type_traits>
#include <cstdint>
#include <string>

#include <boost/preprocessor.hpp>

#if defined(_MSC_VER)
#define __PRETTY_FUNCTION__ __FUNCDNAME__
#endif

#include <cnr_logger/cnr_logger.h>
#include <cnr_logger/cnr_logger_color_macros.h>
#include <cnr_logger/cnr_logger_static_macros.h>

//! Shortcut
namespace cl = cnr_logger;

// ================= STANDARD
#define CNR_FATAL(...) BOOST_PP_OVERLOAD(CNR_FATAL_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_ERROR(...) BOOST_PP_OVERLOAD(CNR_ERROR_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_WARN(...) BOOST_PP_OVERLOAD(CNR_WARN_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_INFO(...) BOOST_PP_OVERLOAD(CNR_INFO_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_INFO_ONLY_FILE(...) BOOST_PP_OVERLOAD(CNR_INFO_ONLY_FILE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_DEBUG(...) BOOST_PP_OVERLOAD(CNR_DEBUG_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_TRACE(...) BOOST_PP_OVERLOAD(CNR_TRACE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_FATAL_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_FATAL_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_ERROR_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_ERROR_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_WARN_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_WARN_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_INFO_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_INFO_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_DEBUG_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_DEBUG_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_TRACE_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_TRACE_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_FATAL_COND(...) BOOST_PP_OVERLOAD(CNR_FATAL_COND_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_ERROR_COND(...) BOOST_PP_OVERLOAD(CNR_ERROR_COND_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_WARN_COND(...) BOOST_PP_OVERLOAD(CNR_WARN_COND_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_INFO_COND(...) BOOST_PP_OVERLOAD(CNR_INFO_COND_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_DEBUG_COND(...) BOOST_PP_OVERLOAD(CNR_DEBUG_COND_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_TRACE_COND(...) BOOST_PP_OVERLOAD(CNR_TRACE_COND_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_FATAL_COND_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_FATAL_COND_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_ERROR_COND_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_ERROR_COND_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_WARN_COND_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_WARN_COND_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_INFO_COND_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_INFO_COND_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_DEBUG_COND_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_DEBUG_COND_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_TRACE_COND_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_TRACE_COND_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_OK(...) BOOST_PP_OVERLOAD(CNR_RETURN_OK_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_NOTOK(...) BOOST_PP_OVERLOAD(CNR_RETURN_NOTOK_, __VA_ARGS__)(__VA_ARGS__)

// ARGS: LOGGER (mandatory), TRUE/FALSE (mandatory), MSG (optional)
#define CNR_RETURN_VOID(...) BOOST_PP_OVERLOAD(CNR_RETURN_VOID_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_OK_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_RETURN_OK_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_OK_THROTTLE_DEFAULT(...) BOOST_PP_OVERLOAD(CNR_RETURN_OK_THROTTLE_DEFAULT_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_NOTOK_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_RETURN_NOTOK_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_NOTOK_THROTTLE_DEFAULT(...)                                                                         \
  BOOST_PP_OVERLOAD(CNR_RETURN_NOTOK_THROTTLE_DEFAULT_, __VA_ARGS__)(__VA_ARGS__)

// ============================== IN/OUT Functions
#define CNR_TRACE_START(...) BOOST_PP_OVERLOAD(CNR_TRACE_START_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_TRACE_END(...) BOOST_PP_OVERLOAD(CNR_TRACE_END_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], True OR False [mandatory], message [optional]
 */
#define CNR_RETURN_BOOL(...) BOOST_PP_OVERLOAD(CNR_RETURN_BOOL_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_RETURN_TRUE(...) BOOST_PP_OVERLOAD(CNR_RETURN_TRUE_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_RETURN_FALSE(...) BOOST_PP_OVERLOAD(CNR_RETURN_FALSE_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_RETURN_FATAL(...) BOOST_PP_OVERLOAD(CNR_RETURN_FATAL_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], ok [mandatory] message [optional]
 */
#define CNR_EXIT_EX(...) BOOST_PP_OVERLOAD(CNR_EXIT_EX_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], time [optional], message [optional]
 */
#define CNR_TRACE_START_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_TRACE_START_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], time [optional], message [optional]
 */
#define CNR_TRACE_START_THROTTLE_DEFAULT(...)                                                                          \
  BOOST_PP_OVERLOAD(CNR_TRACE_START_THROTTLE_DEFAULT_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], True OR False [mandatory], time [optional], message [optional]
 */
#define CNR_RETURN_BOOL_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_RETURN_BOOL_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], time [optional], message [optional]
 */
#define CNR_RETURN_TRUE_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_RETURN_TRUE_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

/**
 * @input: logger [mandatory], time [optional], message [optional]
 */
#define CNR_RETURN_FALSE_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_RETURN_FALSE_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

// ============================== IN/OUT Functions THROTTLE DEFAULT
#define CNR_RETURN_FALSE_THROTTLE(...) BOOST_PP_OVERLOAD(CNR_RETURN_FALSE_THROTTLE_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_BOOL_THROTTLE_DEFAULT(...)                                                                          \
  BOOST_PP_OVERLOAD(CNR_RETURN_BOOL_THROTTLE_DEFAULT_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_TRUE_THROTTLE_DEFAULT(...)                                                                          \
  BOOST_PP_OVERLOAD(CNR_RETURN_TRUE_THROTTLE_DEFAULT_, __VA_ARGS__)(__VA_ARGS__)

#define CNR_RETURN_FALSE_THROTTLE_DEFAULT(...)                                                                         \
  BOOST_PP_OVERLOAD(CNR_RETURN_FALSE_THROTTLE_DEFAULT_, __VA_ARGS__)(__VA_ARGS__)
#endif  // CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_H
