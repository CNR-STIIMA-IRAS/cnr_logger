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
 * @file cnr_logger.h
 * @author Nicola Pedrocchi, Alessio Prini 
 * @date 30 Gen 2021
 * @brief File containing TracLogger class definition.
 *
 * The class has been designed to have a logger separated from the standard ros logging functions.
 * It uses the same core library, log4cxx. The main difference consists of that you can enable or disable the
 * logging to screen and/or to file just using a parameter.
 */


#ifndef CNR_LOGGER_CNR_LOGGER_H
#define CNR_LOGGER_CNR_LOGGER_H

#include <map>
#include <string>
#include <iostream>
#include <memory>


#include <log4cxx/logger.h>
#include <cnr_logger/visibility_control.h>

/**
 * @brief cnr_logger main namespace for the package
 */
namespace cnr_logger
{


/**
 * \class TraceLogger cnr_logger.h
 * @brief The TraceLogger class. The class is a wrapper to log4cxx library
 */
class TraceLogger
{
public:
  enum class AppenderType : int {FILE_STREAM=0, CONSOLE_STREAM=1, SYNC_FILE_AND_CONSOLE=2};
  enum class Level : int {FATAL=0, ERROR=1, WARN=2, INFO=3, DEBUG=4, TRACE=5};

  /**
   * @brief TraceLogger. The constructor does not initilize the class. The function init() must be called afterwards.
   */
  CNR_LOGGER_PUBLIC
  explicit TraceLogger()= default;
  
  CNR_LOGGER_PUBLIC
  TraceLogger(const TraceLogger&) = delete;
  
  CNR_LOGGER_PUBLIC
  TraceLogger(TraceLogger&&) noexcept = delete;

  CNR_LOGGER_PUBLIC
  virtual ~TraceLogger();

  /**
   * @brief TraceLogger: The constructor fully initilize the class.
   * @param[IN] logger_id: unique id for the logger.
   * When the class has more than one appender configured, the logger_id is postponed with a "_x" letter to make the
   * appender unique.
   * @param[IN] path: absolute namespace where the initialization parameters are stored.
   * @param[IN] star_header: if the first log is with '***' to make easy to find the start of the logging in the file.
   * @param[IN] default_values: in the case the parameters are not found under the input namespace, the default
   * configuration is loaded. If FALSE, the function returns false if the parameters are not found.
   */
  CNR_LOGGER_PUBLIC
  TraceLogger(const std::string& logger_id, const std::string& path,
                const bool star_header=false, const bool default_values=true, std::string* what=nullptr);
  

  /**
   * @brief init
   * @param logger_id: unique id for the logger. When the class has more than one appender configured, the logger_id
   * is postponed with a "_x" letter to make the appender unique.
   * @param path: absolute namespacewhere the initialization parameters are stored.
   * @param star_header: if the first log is with '***' to make easy to find the start of the logging in the file.
   * @param default_values: in the case the parameters are not found under the input namespace, the default
   * configuration is loaded. If FALSE, the function returns false if the parameters are not found.
   * @param what: if it is different from nullptr, it stores warnings and errors 
   * @return True if correctly initialized
   */
  CNR_LOGGER_PUBLIC
  bool init(const std::string& logger_id, const std::string& path,
              const bool star_header = false, const bool default_values = true, std::string* what = nullptr);

  [[deprecated("Use the init(const std::string&, const std::string&, const bool, const bool, std::string*)")]]
  CNR_LOGGER_PUBLIC
  bool init_logger(const std::string& logger_id, const std::string& path,
              const bool star_header = false, const bool default_values = true, std::string* what = nullptr);

  CNR_LOGGER_PUBLIC
  TraceLogger& operator=(const TraceLogger& rhs);
  
  CNR_LOGGER_PUBLIC
  TraceLogger const & operator=(TraceLogger &&) = delete; 

  /**
   * @brief SyncFileAndScreen logger getter.
   * @return the logger if exist the SyncFileAndScreen Appender level of logginge
   */
  CNR_LOGGER_PUBLIC
  log4cxx::LoggerPtr syncFileAndScreenLogger( );

  /**
   * @brief File logger getter.
   * @return the logger if exist the File Appender level of logginge
   */
  CNR_LOGGER_PUBLIC
  log4cxx::LoggerPtr fileLogger( );

  /**
   * @brief Console logger getter.
   * @return the logger if exist the Console Appender level of logginge
   */
  CNR_LOGGER_PUBLIC
  log4cxx::LoggerPtr consoleLogger( );

  /**
   * @brief Configuration getter.
   * @return True if the logger has an appender linked to a file, false otherwise
   */
  CNR_LOGGER_PUBLIC
  bool logFile();
  /**
   * @brief Configuration getter.
   * @return True if the logger has an appender linked to a the console, false otherwise
   */
  CNR_LOGGER_PUBLIC
  bool logScreen();

   /**
   * @brief Configuration getter.
   * @return True if the logger has an appender linked only to a file, false otherwise
   */
  CNR_LOGGER_PUBLIC
  bool logOnlyFile();
  /**
   * @brief Configuration getter.
   * @return True if the logger has an appender linked only to a the console, false otherwise
   */
  CNR_LOGGER_PUBLIC
  bool logOnlyScreen();

  /**
   * @brief Configuration getter.
   * @return True if the logger has an appender linked to both the console and the file, false otherwise
   */
  CNR_LOGGER_PUBLIC
  bool logSyncFileAndScreen();

  CNR_LOGGER_PUBLIC
  const double& defaultThrottleTime() const;

  CNR_LOGGER_PUBLIC
  bool logFatal() const;
  
  CNR_LOGGER_PUBLIC
  bool logError() const;
  
  CNR_LOGGER_PUBLIC
  bool logWarn()  const;
  
  CNR_LOGGER_PUBLIC
  bool logInfo()  const;
  
  CNR_LOGGER_PUBLIC
  bool logDebug() const;
  
  CNR_LOGGER_PUBLIC
  bool logTrace() const;

  CNR_LOGGER_PUBLIC
  const std::string& defaultMessage() const;

  CNR_LOGGER_PUBLIC
  const Level& getLevel() const;

  friend std::string to_string(const TraceLogger& logger);
  friend std::ofstream& operator<<(std::ofstream& out, const TraceLogger& logger);

private:
  bool check(const std::string& path) const;

  bool initialized_ = false;
  double default_throttle_time_ = 0.0;
  Level max_level_ = TraceLogger::Level::INFO;
  std::string logger_id_ ="";
  std::string path_ = "";
  std::string default_message_ = "..."; 
  bool default_values_ = false;


  std::map<AppenderType, log4cxx::LoggerPtr> loggers_;
  std::map<AppenderType, Level> levels_;

};

using  TraceLoggerPtr = std::shared_ptr< TraceLogger >;

std::string to_string(const TraceLogger& logger);
std::ostream& operator<<(std::ostream& out, const TraceLogger& logger);


}  // namespace cnr_logger


#include <cnr_logger/cnr_logger_macros.h>  //NOLINT(build/include)

#endif  // CNR_LOGGER_CNR_LOGGER_H
