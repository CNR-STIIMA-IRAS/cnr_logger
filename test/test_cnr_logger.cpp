/**
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

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "cnr_logger_test");
  ros::NodeHandle nh;

  cnr_logger::TraceLogger logger("log1", "/");
  cnr_logger::TraceLogger logger2("log2", "/pippo");

  while (ros::ok())
  {
//    ROS_INFO("Ciao-ros-info");
//    ROS_DEBUG("Ciao-ros-debug");

    CNR_INFO(logger, "Ciao-log-1-info");
    CNR_DEBUG(logger, "Ciao-log-1-debug");

//    CNR_INFO_THROTTLE (logger,5, "*** THROTTLE *** Ciao-log-1-info");
//    CNR_DEBUG_THROTTLE (logger,5, "*** THROTTLE *** Ciao-log-1-debug");

    CNR_INFO(logger2, "Ciao-log-2-info");
    CNR_DEBUG(logger2, "Ciao-log-2-debug");

    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  return 1;
}
