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

/**
 * @file cnr_logger.h
 * @author Nicola Pedrocchi
 * @date 25 Jun 2020
 * @brief File containing TracLogger class definition.
 *
 * The class has been designed to have a logger separated from the standard ros logging functions. It uses the same core library, log4cxx. The main difference consists of that you can enable or disable the logging to screen and/or to file just using a parameter.
 */


#ifndef CNR_LOGGER_CNR_LOGGER_H
#define CNR_LOGGER_CNR_LOGGER_H

#include <map>
#include <string>
#include <iostream>
#include <ros/file_log.h>
#include <ros/console.h>

#include <log4cxx/logger.h>
#include <log4cxx/basicconfigurator.h>
#include <log4cxx/propertyconfigurator.h>
#include <log4cxx/helpers/exception.h>
#include <log4cxx/rollingfileappender.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <cnr_logger/cnr_logger_macros.h>  //NOLINT(build/include)

/**
 * @brief cnr_logger main namespace for the package
 */
namespace cnr_logger
{

/**
 * @brief Utility to print nicely the time.
 * @param now Time to translate in string.
 */
inline std::string to_string(const ros::Time& now)
{
  auto current_time = now.toBoost();
  std::stringstream ss;
  auto facet = new boost::posix_time::time_facet("%Y%m%d-%H:%M:%s");
  ss.imbue(std::locale(std::locale::classic(), facet));
  ss << current_time;
  return ss.str();
}

/**
 * \class TraceLogger cnr_logger.h
 * @brief The TraceLogger class. The class is a wrapper to log4cxx library
 */
class TraceLogger
{
public:
  enum AppenderType { FILE_STREAM = 0, CONSOLE_STREAM = 1, SYNC_FILE_AND_CONSOLE = 2 };

  /**
   * @brief TraceLogger. The constructor does not initilize the class. THe function init() must be called afterwards.
   * @param logger_id: unique id for the logger.
   * When the class has more than one appender configured, the logger_id is postponed with a "_x" letter to make the appender unique.
   */
  explicit TraceLogger(const std::string& logger_id);
  /**
   * @brief TraceLogger: The constructor fully initilize the class.
   * @param logger_id: unique id for the logger.
   * When the class has more than one appender configured, the logger_id is postponed with a "_x" letter to make the appender unique.
   * @param param_namespace: absolute namespacewhere the initialization parameters are stored.
   * @param star_header: if the first log is with '***' to make easy to find the start of the logging in the file.
   */
  TraceLogger(const std::string& logger_id, const std::string& param_namespace, const bool star_header = false);
  ~TraceLogger();

  /**
   * @brief init
   * * @param logger_id: unique id for the logger. When the class has more than one appender configured, the logger_id is postponed with a "_x" letter to make the appender unique.
   * @param param_namespace: absolute namespacewhere the initialization parameters are stored.
   * @param star_header: if the first log is with '***' to make easy to find the start of the logging in the file.
     @param default_values: in the case the parameters are not found under the input namespace, the default configuration is loaded. If FALSE, the function returns false if the parameters are not found.
   * @return True if correctly initialized
   */
  bool init(const std::string& param_namespace, const bool star_header = false, const bool default_values = true);


  /**
   * @brief Configuration getter.
   * @return True if the logger has an appender linked to a file, false otherwise
   */
  bool logFile()
  {
    return (loggers_.find(FILE_STREAM) != loggers_.end());
  }
  /**
   * @brief Configuration getter.
   * @return True if the logger has an appender linked to a the console, false otherwise
   */
  bool logScreen()
  {
    return (loggers_.find(CONSOLE_STREAM)        != loggers_.end());
  }

  /**
   * @brief Configuration getter.
   * @return True if the logger has an appender linked to both the console and the file, false otherwise
   */
  bool logSyncFileAndScreen()
  {
    return (loggers_.find(SYNC_FILE_AND_CONSOLE) != loggers_.end());
  }

  std::map< AppenderType, log4cxx::LoggerPtr > loggers_;
  std::map< AppenderType, std::string        > levels_;
  const std::string logger_id_;

private:
  bool check(const std::string& param_namespace);
  bool initialized_;
};

typedef std::shared_ptr< TraceLogger > TraceLoggerPtr;

}  // namespace cnr_logger

#endif  // CNR_LOGGER_CNR_LOGGER_H
