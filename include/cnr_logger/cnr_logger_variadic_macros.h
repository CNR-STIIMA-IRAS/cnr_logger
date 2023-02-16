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
#ifndef CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_H
#define CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_H

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
#include <cnr_logger/cnr_logger_variadic_macros_support.h>


//! Shortcut
namespace cl = cnr_logger;

// ============================== IN/OUT Functions
#if 0

/**
 *
 */
#define CNR_TRACE_START(logger, ...)\
do\
{\
  if((std::string(__VA_ARGS__).length() > 0))\
    CNR_TRACE(logger, cl::YELLOW() + "[ START] " +cl::RESET() << std::string(__VA_ARGS__) );\
  else\
    CNR_TRACE(logger, cl::YELLOW() + "[ START] " +cl::RESET() << __FUNCTION__);\
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
  std::string result = (ret ? cl::GREEN()+"[  DONE] " : cl::RED()+"[FAILED] ") + cl::RESET();\
  if((std::string(__VA_ARGS__).length() > 0))\
    CNR_TRACE(logger, result << std::string(__VA_ARGS__) );\
  else\
    CNR_TRACE(logger, result << __FUNCTION__);\
} while (false);\
return ret;\

/**
 *
 */
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
    CNR_TRACE(logger, (cl::RED() + "[FAILED] " + cl::RESET())<< __FUNCTION__);\
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
  CNR_TRACE(logger, cl::RED() + "[FAILED] " + cl::RESET() << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_VOID(logger, ok, ...)\
  if(ok)\
  {\
    CNR_RETURN_OK(logger,void(),__VA_ARGS__);\
  }\
  else\
  {\
    CNR_RETURN_NOTOK(logger,void(),__VA_ARGS__);\
  }

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
    CNR_TRACE(logger, cl::RED() + "[FAILED] " + cl::RESET() << __FUNCTION__);\
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
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, (ret ? "[  DONE] " : cl::RED() + "[FAILED] " + cl::RESET())\
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
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, cl::RED() + "[FAILED] " + cl::RESET() << __FUNCTION__);\
} while (false);\
return(var);




// ============================== IN/OUT Functions THROTTLE DEFAULT
#define CNR_TRACE_START_THROTTLE_DEFAULT(logger, ...)\
do\
{\
  double period = cl::getTraceLogger(logger) ? cl::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__)); \
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, "[ START] " << __FUNCTION__);\
} while (false)

#define CNR_RETURN_BOOL_THROTTLE_DEFAULT(logger, ret, ...)\
do\
{\
  double period = cl::getTraceLogger(logger) ? cl::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  if (ret) { CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__)); }\
  else     { CNR_ERROR_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));}\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, (ret ? "[  DONE] " : cl::RED() + "[FAILED] " + cl::RESET())\
                                    << __FUNCTION__);\
} while (false);\
return ret;

#define CNR_RETURN_TRUE_THROTTLE_DEFAULT(logger,  ...)\
do\
{\
  double period = cl::getTraceLogger(logger) ? cl::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_RETURN_BOOL_THROTTLE(logger, true, period, __VA_ARGS__);\
} while(false);\


#define CNR_RETURN_FALSE_THROTTLE_DEFAULT(logger, ...)\
do\
{\
  double period = cl::getTraceLogger(logger) ? cl::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_RETURN_BOOL_THROTTLE(logger, false, period, __VA_ARGS__);\
} while(false);\

#define CNR_RETURN_OK_THROTTLE_DEFAULT(logger, var, ...)\
do\
{\
  double period = cl::getTraceLogger(logger) ? cl::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_INFO_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, "[  DONE] " << __FUNCTION__);\
} while (false);\
return(var);

#define CNR_RETURN_NOTOK_THROTTLE_DEFAULT(logger, var, ...)\
do\
{\
  double period = cl::getTraceLogger(logger) ? cl::getTraceLogger(logger)->defaultThrottleTime() : 1.0;\
  CNR_ERROR_COND_THROTTLE(logger, std::string(__VA_ARGS__).length() > 0, period, std::string(__VA_ARGS__));\
  CNR_TRACE_COND_THROTTLE(logger, period>0, period, cl::RED() + "[FAILED] " + cl::RESET() << __FUNCTION__);\
} while (false);\
return(var);

#else

// ============================== IN/OUT Functions
/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_TRACE_START(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  CNR_TRACE(e.logger_, \
    cl::YELLOW() + "[ START] " + cl::RESET() << ( e.msg_.empty() ? __FUNCTION__ : e.msg_) );\
} while (false)

/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_TRACE_END(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  CNR_INFO_COND(e.logger_, !e.msg_.empty(), e.msg_); \
  CNR_TRACE(e.logger_, "[  DONE] " << __FUNCTION__);\
} while (false)

/**
 * @input: logger [mandatory], True OR False [mandatory], message [optional]
 */
#define CNR_RETURN_BOOL(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  std::string result = (e.ok_ ? cl::GREEN()+"[  DONE] " : cl::RED()+"[FAILED] ") + cl::RESET();\
  CNR_TRACE(e.logger_, result << (e.msg_.empty() ? __FUNCTION__ : e.msg_ ) );\
  return(e.ok_);\
} while (false);\


/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_RETURN_TRUE(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  \
  std::string msg = cl::GREEN()+"[  DONE] " + cl::RESET() + (e.msg_.empty() ? __FUNCTION__ : e.msg_ );\
  CNR_TRACE(e.logger_, msg);\
} while (false);\
return(true);

/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_RETURN_FALSE(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  \
  std::string msg = cl::RED()+"[FAILED] " + cl::RESET() + (e.msg_.empty() ? __FUNCTION__ : e.msg_ );\
  CNR_TRACE(e.logger_, msg );\
} while (false);\
return(false);

/**
 * @input: logger [mandatory], message [optional]
 */
#define CNR_RETURN_FATAL(...)\
  do\
  {\
    cl::VariadicParser e(__VA_ARGS__);\
    \
    CNR_FATAL_COND(e.logger_, e.msg_.length(), e.msg_);\
    CNR_TRACE(e.logger_, (cl::RED() + "[FAILED] " + cl::RESET())<< __FUNCTION__);\
  } while (false);\
  return false;\

/**
 * @input: logger [mandatory], message [optional]
 */

#define CNR_EXIT_EX(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
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

/**
 * @input: logger [mandatory], time [optional], message [optional]
 */
#define CNR_TRACE_START_THROTTLE(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_); \
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, "[ START] " << __FUNCTION__);\
} while (false)

/**
 * @input: logger [mandatory], True OR False [mandatory], time [optional], message [optional]
 */
#define CNR_RETURN_BOOL_THROTTLE(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  if (e.ok_){ CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_); }\
  else      { CNR_ERROR_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_);}\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, \
      (e.ok_ ? "[  DONE] " : cl::RED() + "[FAILED] " + cl::RESET()) << __FUNCTION__);\
  return(e.ok_);\
} while (false);\


/**
 * @input: logger [mandatory], time [optional], message [optional]
 */
#define CNR_RETURN_TRUE_THROTTLE(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_);\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, ("[  DONE] "+cl::RESET()) << __FUNCTION__);\
} while (false);\
return(true);

/**
 * @input: logger [mandatory], time [optional], message [optional]
 */
#define CNR_RETURN_FALSE_THROTTLE(...)\
do\
{\
  cl::VariadicParser e; e.pp_get(__VA_ARGS__);\
  CNR_ERROR_COND_THROTTLE(e.logger_, e.msg_.length(), e.period_, e.msg_);\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, ("[  DONE] "+cl::RESET()) << __FUNCTION__);\
} while (false);\
return(false);



// ============================== IN/OUT Functions THROTTLE DEFAULT
#define CNR_TRACE_START_THROTTLE_DEFAULT(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  e.default_period();\
  CNR_INFO_COND_THROTTLE(e.logger_, e.msg_.length() > 0, e.period_, e.msg_); \
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, "[ START] " << __FUNCTION__);\
} while (false)

#define CNR_RETURN_BOOL_THROTTLE_DEFAULT(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
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
  cl::VariadicParser e(__VA_ARGS__);\
  e.default_period();\
  CNR_INFO_COND_THROTTLE( e.logger_, e.msg_.length() > 0, e.period_, e.msg_);\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, "[  DONE] " << __FUNCTION__);\
  return(true);\
} while(false);\


#define CNR_RETURN_FALSE_THROTTLE_DEFAULT(...)\
do\
{\
  cl::VariadicParser e(__VA_ARGS__);\
  e.default_period();\
  CNR_ERROR_COND_THROTTLE(e.logger_, e.msg_.length() > 0, e.period_, e.msg_);\
  CNR_TRACE_COND_THROTTLE(e.logger_, e.period_>0, e.period_, "[  DONE] " << __FUNCTION__);\
  return(false);\
} while(false);\


#endif

#endif  // CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_H
