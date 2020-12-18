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
#include <gtest/gtest.h>

std::shared_ptr<cnr_logger::TraceLogger> logger;
std::shared_ptr<cnr_logger::TraceLogger> logger2;

TEST(TestSuite, colorTest)
{
  std::cout << "Color Test" << std::endl;
  std::cout << cnr_logger::RST() << "RESET" << " ";
  std::cout << cnr_logger::BLK() << "BLACK" << "|";
  std::cout << cnr_logger::R() << "RED" << "|";
  std::cout << cnr_logger::G() << "GREEN" << "|";
  std::cout << cnr_logger::Y() << "YELLOW" << "|";
  std::cout << cnr_logger::BLE() << "BLUE" << "|";
  std::cout << cnr_logger::M() << "MAGENTA" << "|";
  std::cout << cnr_logger::C() << "CYAN" << "|";
  std::cout << cnr_logger::W() << "WHITE" << "|";
  std::cout << cnr_logger::BBLK() << "BOLDBLACK" << "|";
  std::cout << cnr_logger::BR() << "BOLDRED" << "|";
  std::cout << cnr_logger::BG() << "BOLDGREEN" << "|";
  std::cout << cnr_logger::BY() << "BOLDYELLOW" << "|";
  std::cout << cnr_logger::BBLE() << "BOLDBLUE" << "|";
  std::cout << cnr_logger::BM() << "BOLDMAGENTA" << "|";
  std::cout << cnr_logger::BC() << "BOLDCYAN" << "|";
  std::cout << cnr_logger::BW() << "BOLDWHITE" << "|";
  std::cout << cnr_logger::RST() << std::endl;
}


// Declare a test
TEST(TestSuite, fullConstructor)
{

  EXPECT_NO_FATAL_FAILURE(logger.reset(new cnr_logger::TraceLogger("log1", "/file_and_screen_different_appenders")));
  EXPECT_FALSE(logger->init("log1", "/file_and_screen_different_appenders", false, false));  // Already initialized

  EXPECT_NO_FATAL_FAILURE(logger2.reset(new cnr_logger::TraceLogger("log2", "/file_and_screen_same_appender")));

  EXPECT_TRUE(logger->logFile());
  EXPECT_TRUE(logger->logScreen());
  EXPECT_FALSE(logger->logSyncFileAndScreen());

  EXPECT_FALSE(logger2->logFile());
  EXPECT_FALSE(logger2->logScreen());
  EXPECT_TRUE(logger2->logSyncFileAndScreen());

  EXPECT_NO_FATAL_FAILURE(logger.reset());
  EXPECT_NO_FATAL_FAILURE(logger2.reset());
}

TEST(TestSuite, partialConstructor)
{
  EXPECT_NO_FATAL_FAILURE(logger.reset(new cnr_logger::TraceLogger()));
  EXPECT_TRUE(logger->init("log1", "/file_and_screen_different_appenders", false, false));
  EXPECT_FALSE(logger->init("log1", "/file_and_screen_different_appenders", false, false));  // Already initialized
  EXPECT_NO_FATAL_FAILURE(logger.reset());
}

TEST(TestSuite, wrongConstructor)
{
  EXPECT_NO_FATAL_FAILURE(logger.reset(new cnr_logger::TraceLogger( )));
  EXPECT_FALSE(logger->init("log1", "/this_namespace_does_not_exist", false, false));
  EXPECT_TRUE(logger->init("log1", "/this_namespace_does_not_exist", false, true));   // default vaules depiste none parameters
  // exits
  EXPECT_NO_FATAL_FAILURE(logger.reset());
}

// Declare another test
TEST(TestSuite, flushFileAndScreen)
{
  EXPECT_NO_FATAL_FAILURE(logger.reset(new cnr_logger::TraceLogger("log1", "/file_and_screen_same_appender", true)));

  for (size_t i = 0; i < 10; i++)
  {
    CNR_INFO(*logger, "Ciao-log-1-info");
    CNR_DEBUG(*logger, "Ciao-log-1-debug");
    CNR_FATAL(*logger, "Ciao-log-1-fatal");
    CNR_TRACE(*logger, "Ciao-log-1-trace");

    CNR_INFO_THROTTLE(*logger,  1.0, "Ciao-log-1-info");
    CNR_DEBUG_THROTTLE(*logger, 1.0, "Ciao-log-1-debug");
    CNR_FATAL_THROTTLE(*logger, 1.0, "Ciao-log-1-fatal");
    CNR_TRACE_THROTTLE(*logger, 1.0, "Ciao-log-1-trace");

    CNR_INFO_COND(*logger, true, "Ciao-log-1-info");
    CNR_DEBUG_COND(*logger, false, "Ciao-log-1debug");
    CNR_FATAL_COND(*logger, true, "Ciao-log-1fatal");
    CNR_TRACE_COND(*logger, false, "Ciao-log-1trace");

    CNR_INFO_COND_THROTTLE(*logger, true, 1.0, "Ciao-log-1-info");
    CNR_DEBUG_COND_THROTTLE(*logger, false, 1.0, "Ciao-log-1-debug");
    CNR_FATAL_COND_THROTTLE(*logger, true, 1.0, "Ciao-log-1-fatal");
    CNR_TRACE_COND_THROTTLE(*logger, false, 1.0, "Ciao-log-1-trace");

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  EXPECT_NO_FATAL_FAILURE(logger.reset());
}

// Declare another test
TEST(TestSuite, flushInfoDebug)
{
  EXPECT_NO_FATAL_FAILURE(logger.reset(new cnr_logger::TraceLogger("log1", "/only_file_streamer", true)));
  EXPECT_NO_FATAL_FAILURE(logger2.reset(new cnr_logger::TraceLogger("log2", "/file_and_screen_same_appender")));

  for (size_t i = 0; i < 10; i++)
  {
    CNR_INFO(*logger, "Ciao-log-1-info");
    CNR_DEBUG(*logger, "Ciao-log-1-debug");
    CNR_FATAL(*logger, "Ciao-log-1-fatal");
    CNR_TRACE(*logger, "Ciao-log-1-trace");

    CNR_INFO_THROTTLE(*logger,  1.0, "Ciao-log-1-info");
    CNR_DEBUG_THROTTLE(*logger, 1.0, "Ciao-log-1-debug");
    CNR_FATAL_THROTTLE(*logger, 1.0, "Ciao-log-1-fatal");
    CNR_TRACE_THROTTLE(*logger, 1.0, "Ciao-log-1-trace");

    CNR_INFO_COND(*logger, true, "Ciao-log-1-info");
    CNR_DEBUG_COND(*logger, false, "Ciao-log-1debug");
    CNR_FATAL_COND(*logger, true, "Ciao-log-1fatal");
    CNR_TRACE_COND(*logger, false, "Ciao-log-1trace");

    CNR_INFO_COND_THROTTLE(*logger, true, 1.0, "Ciao-log-1-info");
    CNR_DEBUG_COND_THROTTLE(*logger, false, 1.0, "Ciao-log-1-debug");
    CNR_FATAL_COND_THROTTLE(*logger, true, 1.0, "Ciao-log-1-fatal");
    CNR_TRACE_COND_THROTTLE(*logger, false, 1.0, "Ciao-log-1-trace");

    CNR_INFO(*logger2, "Ciao-log-2-info");
    CNR_DEBUG(*logger2, "Ciao-log-2-debug");

    ros::spinOnce();
    ros::Duration(0.1).sleep();
  }
  EXPECT_NO_FATAL_FAILURE(logger.reset());
  EXPECT_NO_FATAL_FAILURE(logger2.reset());
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "cnr_logger_tester");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
