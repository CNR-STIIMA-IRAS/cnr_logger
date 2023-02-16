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
#ifndef CNR_LOGGER_CNR_LOGGER_COLOR_MACROS_H
#define CNR_LOGGER_CNR_LOGGER_COLOR_MACROS_H

#include <string>


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

}  // namespace cnr_logger

#endif // CNR_LOGGER_CNR_LOGGER_COLOR_MACROS_H
