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

#include <iostream>
#include <ros/ros.h>
#include <cnr_logger/cnr_logger.h>

std::shared_ptr<cnr_logger::TraceLogger> logger;

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv,"cnr_logger_example");
  ros::NodeHandle nh;

  std::cout << cnr_logger::BOLDCYAN() << "START" << cnr_logger::RESET() << std::endl;

  logger.reset(new cnr_logger::TraceLogger("log1", "/file_and_screen_same_appender", true));

  for (size_t i = 0; i < 10; i++)
  {
    CNR_INFO( *logger, "Ciao-log-1-info");
    CNR_DEBUG(*logger, "Ciao-log-1-debug");
    CNR_FATAL(*logger, "Ciao-log-1-fatal");
    CNR_TRACE(*logger, "Ciao-log-1-trace");

    CNR_INFO_THROTTLE(*logger,  1.0, "Ciao-log-1-info");
    CNR_DEBUG_THROTTLE(*logger, 1.0, "Ciao-log-1-debug");
    CNR_FATAL_THROTTLE(*logger, 1.0, "Ciao-log-1-fatal");
    CNR_TRACE_THROTTLE(*logger, 1.0, "Ciao-log-1-trace");

    CNR_INFO_COND( *logger, true , "Ciao-log-1-info");
    CNR_DEBUG_COND(*logger, false, "Ciao-log-1debug");
    CNR_FATAL_COND(*logger, true , "Ciao-log-1fatal");
    CNR_TRACE_COND(*logger, false, "Ciao-log-1trace");

    CNR_INFO_COND_THROTTLE( *logger, true , 1.0, "Ciao-log-1-info");
    CNR_DEBUG_COND_THROTTLE(*logger, false, 1.0, "Ciao-log-1-debug");
    CNR_FATAL_COND_THROTTLE(*logger, true , 1.0, "Ciao-log-1-fatal");
    CNR_TRACE_COND_THROTTLE(*logger, false, 1.0, "Ciao-log-1-trace");

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  logger.reset();
  std::cout << cnr_logger::BOLDGREEN() << "OK" << cnr_logger::RESET() << std::endl;
  return 0;
}
