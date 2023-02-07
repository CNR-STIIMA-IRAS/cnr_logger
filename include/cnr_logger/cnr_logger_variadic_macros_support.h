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
 * @file cnr_logger_color_macros.h
 * @author Nicola Pedrocchi, ALessio Prini
 * @date 07 Feb 2022
 * @brief File containing the definition of the macro for the colors management.
 *
 * The macro have been designed to follow the basic log4cxx structure (and ROS).
 * The further macro CNR_TRACE_START and CNR_RETURN_xxx are used to trace the input and the output values of the functions
 */
#ifndef CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_SUPPORT_H
#define CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_SUPPORT_H

#include <string>
#include <cnr_logger/cnr_logger.h>

namespace cnr_logger
{
    
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

    
}  // namespace cnr_logger

#endif // CNR_LOGGER_CNR_LOGGER_VARIADIC_MACROS_SUPPORT_H


