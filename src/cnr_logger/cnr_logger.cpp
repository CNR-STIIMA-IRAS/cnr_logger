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

#include <string>
#include <vector>
#include <iostream>
#include <cnr_logger/cnr_logger.h>

#if defined(ROS_NOT_AVAILABLE)
  #include <yaml-cpp/yaml.h>
#else
  #include <ros/common.h>
  #if ROS_VERSION_MINIMUM(1, 14, 1)
    #include <ros/file_log.h>
    #include <ros/console.h>
    #include <ros/param.h>
    #include <boost/date_time/posix_time/posix_time.hpp>
    #include <boost/date_time/posix_time/posix_time_io.hpp>
  #else
    #error "The minimum ros version 1.14.1 is not satisfied"
  #endif
#endif

#include <fstream>
#include <pwd.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>

std::string mkLogDir()
{
  struct passwd *pw = getpwuid(getuid());
  const char *homedir = pw->pw_dir;
  std::string homedir_string = std::string(homedir) + "/.local/log";
  DIR* dir = opendir(homedir_string.c_str());
  if (dir)
  {
    closedir(dir);
  }
  else if (ENOENT == errno)
  {
    mkdir(homedir_string.c_str(),0777);
  }
  return homedir_string;
}

using log4cxx::ColorPatternLayout;
IMPLEMENT_LOG4CXX_OBJECT(ColorPatternLayout);

namespace cnr_logger
{

/**
 * @brief Utility to print nicely the time.
 * @param now Time to translate in string.
 */
#if defined(ROS_NOT_AVAILABLE)
std::string to_string(const time_t& now)
#else
std::string to_string(const ros::Time& now)
#endif
{
  std::string ret;
#if defined(ROS_NOT_AVAILABLE)
  char* date_time = ctime(&now);
  ret = std::string(date_time);
#else
#if ROS_VERSION_MINIMUM(1, 14, 1)
  auto current_time = now.toBoost();
  std::stringstream ss;
  auto facet = new boost::posix_time::time_facet("%Y%m%d-%H:%M:%s");
  ss.imbue(std::locale(std::locale::classic(), facet));
  ss << current_time;
  ret = ss.str();
#else
  #error "The minimum version of ros is 1.14.1"
#endif
#endif
  return ret;
}


template<typename T>
#if defined(ROS_NOT_AVAILABLE)
bool extract(T& val,
              const YAML::Node& path,
                const std::string& leaf,
                    const T& default_val)
#else
bool extract(T& val,
              const std::string& path,
                const std::string& leaf,
                  const T& default_val)
#endif
{
  bool ret = false;
#if defined(ROS_NOT_AVAILABLE)
  ret = path[leaf];
#else
  ret = ros::param::get(path + "/" + leaf, val);
#endif
  if(!ret)
  {
    val = default_val;
  }
#if defined(ROS_NOT_AVAILABLE)
  else
  {
    val = path[leaf].as<T>();
  }
#endif
  return ret;
}

#if defined(ROS_NOT_AVAILABLE)
bool extractVector(std::vector<std::string>& val,
              const YAML::Node& path,
                const std::string& leaf,
                    const std::vector<std::string>& default_val)
#else
bool extractVector(std::vector<std::string>& val,
                      const std::string& path,
                        const std::string& leaf,
                          const std::vector<std::string>& default_val)
#endif
{
  bool ret = false;
#if defined(ROS_NOT_AVAILABLE)
  ret = path[leaf];
#else
  ret = ros::param::get(path + "/" + leaf, val);
#endif
  if(!ret && default_val.size())
  {
    val.resize(default_val.size());
    std::copy(default_val.begin(), default_val.end(), val.begin());
  }
#if defined(ROS_NOT_AVAILABLE)
  else
  {
    for(YAML::const_iterator it=path[leaf].begin();it!=path[leaf].end();++it)
    {
      val.push_back(it->as<std::string>());
    }
   }
#endif
  return ret;
}

TraceLogger::TraceLogger()
  : logger_id_(""), path_(""), default_values_(false), initialized_(false)
{
}

TraceLogger::TraceLogger(const std::string& logger_id, const std::string& path,
                         const bool star_header, const bool default_values)
  : TraceLogger()
{
  try
  {
    if(!init(logger_id, path, star_header, default_values))
    {
      std::cerr << logger_id << ": Error in creating the TraceLogger. " << std::endl;
    }
  }
  catch (std::exception& e)
  {
    std::cerr << logger_id << ": Error in creating the TraceLogger. Exception: " << e.what() << std::endl;
  }
  catch (...)
  {
    std::cerr << logger_id << ": Error in creating the TraceLogger. Unhandled Exception" << std::endl;
  }
}


bool TraceLogger::check(const std::string& path)
{
  bool res = true;
#if defined(ROS_NOT_AVAILABLE)
  YAML::Node config = YAML::LoadFile(path);

  res &= (config["appenders"] && config["appenders"].IsSequence());
  res &= (config["levels"]?config["levels"].IsSequence():true);
  res &= (config["pattern_layout"]?config["pattern_layout"].IsScalar():true);
  res &= (config["file_name"]?config["file_name"].IsScalar():true);
  res &= (config["append_date_to_file_name"]?config["append_date_to_file_name"].IsScalar():true);
  res &= (config["append_to_file"]?config["append_to_file"].IsScalar():true);
#else
  res = ros::param::has(path + "/appenders")
         || ros::param::has(path + "/levels")
         || ros::param::has(path + "/pattern_layout")
         || ros::param::has(path + "/file_name")
         || ros::param::has(path + "/append_date_to_file_name")
         || ros::param::has(path + "/append_to_file");
#endif
  return res;
}

bool TraceLogger::init(const std::string& logger_id, const std::string& path,
                          const bool star_header, const bool default_values)
{
  if(initialized_)
  {
    std::cerr<< logger_id_ << ": Logger already initialized."<<std::endl;
    return false;
  }

  logger_id_ = logger_id;
  path_ = path;
  default_values_ = default_values;

  if((!default_values) && (!check(path)))
  {
    return false;
  }

  // =======================================================================================
  // Extract the info to constructs the logger
  //
  // =======================================================================================
  std::vector<std::string> appenders, levels;
#if defined(ROS_NOT_AVAILABLE)
  YAML::Node _path = YAML::LoadFile(path);
  auto now = time(0);
#else
  std::string _path = path;
    auto now = ros::Time::now();
#endif

  std::vector<std::string> empty;
  if(!extractVector(appenders, _path, "appenders", empty))
  {
    std::cerr << logger_id_ << ": Paremeter missing: path='"<<path<<"', key='"<<"appenders'"<<std::endl;
  }

  if(!extractVector(levels, _path, "levels", empty))
  {
    std::cerr << logger_id_ << ": Paremeter missing: path='"<<path<<"', key='"<<"levels'"<<std::endl;
  }

  if(appenders.size() != levels.size())
  {
    std::cerr << logger_id_
                << ": Size of appenders and levels mismatch! Default INFO level for all the appenders" << std::endl;
    levels.clear();
    levels.resize(appenders.size(), "DEBUG");
  }


  for(size_t i = 0; i < appenders.size(); i++)
  {
    std::for_each(appenders[i].begin(), appenders[i].end(), [](char & c)
    {
      c = ::tolower(c);
    });
    std::for_each(levels   [i].begin(), levels   [i].end(), [](char & c)
    {
      c = ::toupper(c);
    });
    if(appenders[i] == "file")
    {
      // nothing to do
    }
    else if((appenders[i] == "console") || (appenders[i] == "cout")
      || (appenders[i] == "terminal") || (appenders[i] == "screen") || (appenders[i] == "video"))
    {
      appenders[i] = "screen";
    }
  }

  auto it_file     = std::find(appenders.begin(), appenders.end(), "file");
  auto it_screen   = std::find(appenders.begin(), appenders.end(), "screen");
  int  idx_file    = (it_file   != appenders.end()) ? std::distance(appenders.begin(), it_file) : -1;
  int  idx_screen  = (it_screen != appenders.end()) ? std::distance(appenders.begin(), it_screen) : -1;

  if(((idx_file >= 0) && (idx_screen >= 0)) && (levels[idx_file] == levels[idx_screen]))   // 1 logger and 2 appenders
  {
    loggers_  [ SYNC_FILE_AND_CONSOLE ] = log4cxx::Logger::getLogger(logger_id_);
    levels_   [ SYNC_FILE_AND_CONSOLE ] = levels[idx_file] == "FATAL"     ? FATAL
                                        : levels[idx_file] == "ERROR"     ? ERROR
                                        : levels[idx_file] == "WARN"      ? WARN
                                        : levels[idx_file] == "INFO"      ? INFO
                                        : levels[idx_file] == "DEBUG"     ? DEBUG
                                        : TRACE;
  }
  else
  {
    if(idx_file >= 0)
    {
      loggers_[FILE_STREAM] = log4cxx::Logger::getLogger(logger_id_ + "_f");
      levels_ [FILE_STREAM] = levels[idx_file] == "FATAL"     ? FATAL
                            : levels[idx_file] == "ERROR"     ? ERROR
                            : levels[idx_file] == "WARN"      ? WARN
                            : levels[idx_file] == "INFO"      ? INFO
                            : levels[idx_file] == "DEBUG"     ? DEBUG
                            : TRACE;
    }
    if(idx_screen >= 0)
    {
      loggers_[CONSOLE_STREAM] = log4cxx::Logger::getLogger(logger_id_);
      levels_ [CONSOLE_STREAM] = levels[idx_screen] == "FATAL"     ? FATAL
                               : levels[idx_screen] == "ERROR"     ? ERROR
                               : levels[idx_screen] == "WARN"      ? WARN
                               : levels[idx_screen] == "INFO"      ? INFO
                               : levels[idx_screen] == "DEBUG"     ? DEBUG
                               : TRACE;
    }
  }

  max_level_ = FATAL;
  for(auto const & level : levels_)
  {
    switch(level.second)
    {
      case FATAL: loggers_[level.first]->setLevel(log4cxx::Level::getFatal()); break;
      case ERROR: loggers_[level.first]->setLevel(log4cxx::Level::getError()); break;
      case WARN : loggers_[level.first]->setLevel(log4cxx::Level::getWarn() ); break;
      case INFO : loggers_[level.first]->setLevel(log4cxx::Level::getInfo() ); break;
      case DEBUG: loggers_[level.first]->setLevel(log4cxx::Level::getDebug()); break;
      case TRACE: loggers_[level.first]->setLevel(log4cxx::Level::getTrace()); break;
    }
    max_level_ = level.second >= max_level_ ? level.second : max_level_;
  }
  // =======================================================================================



  // =======================================================================================
  // Layout
  //
  // =======================================================================================
  std::string pattern_layout;
  std::string default_pattern_layout = "[%5p][%d{HH:mm:ss,SSS}][%M:%L][%c] %m%n";
  if(!extract(pattern_layout, _path, "pattern_layout", default_pattern_layout))
  {
    std::cerr << logger_id_ << ": Paremeter missing: path='"<<path<<"', key='"<<"pattern_layout'"<<std::endl;
  }
  log4cxx::ColorPatternLayoutPtr layout = new log4cxx::ColorPatternLayout(pattern_layout);
  // =======================================================================================



  // =======================================================================================
  // Configure appenders and loggers
  //
  // =======================================================================================
  std::string log_file_name;
  std::string root_path = mkLogDir();
  std::string default_log_file_name = log_file_name = root_path + "/" + logger_id_;
  if(logFile() || logSyncFileAndScreen())
  {
    if(!extract(log_file_name, _path, "file_name", default_log_file_name))
    {
      std::cerr << logger_id_ << ": Paremeter missing: path='"<<path<<"', key='"<<"file_name"
                  << ", default superimposed: "<< default_log_file_name <<std::endl;
    }

    bool append_date_to_file_name = false;
    bool default_append_date_to_file_name = false;
    if(!extract(append_date_to_file_name, _path, "append_date_to_file_name", default_append_date_to_file_name))
    {
      std::cerr << logger_id_ << ": Paremeter missing: path='"<<path<<"', key='"<<"append_date_to_file_name"
                  << ", default superimposed: "<< default_append_date_to_file_name <<std::endl;
    }

    log_file_name +=  append_date_to_file_name  ? ("." + to_string(now) + ".log") : ".log";

    bool append_to_file = false;
    bool default_append_to_file = false;
    if(!extract(append_to_file, _path, "append_to_file", default_append_to_file))
    {
      std::cerr << logger_id_ << ": Paremeter missing: path='"<<path<<"', key='"<<"append_to_file"
                  << ", default superimposed: "<< default_append_to_file <<std::endl;
    }

    log4cxx::RollingFileAppenderPtr appender = new log4cxx::RollingFileAppender(layout, log_file_name, append_to_file);
    loggers_[ logFile() ? FILE_STREAM : SYNC_FILE_AND_CONSOLE ]->addAppender(appender);

    appender->setMaximumFileSize(100 * 1024 * 1024);
    appender->setMaxBackupIndex(10);
    log4cxx::helpers::Pool pool;
    appender->activateOptions(pool);
  }

  if(logScreen() || logSyncFileAndScreen())
  {
    log4cxx::ConsoleAppenderPtr appender = new log4cxx::ConsoleAppender(layout, logger_id_);
    loggers_[  logScreen() ? CONSOLE_STREAM : SYNC_FILE_AND_CONSOLE ]->addAppender(appender);
  }

  std::string loggers_str = "n. app.: " + std::to_string(static_cast<int>(loggers_.size())) + " ";
  for(auto const & l : loggers_)
  {
    loggers_str += std::to_string(l.first) + "|";
  }

  loggers_str += (levels_.size() > 0 ?  " l: " : "");
  for(auto const & l : levels_)
  {
    loggers_str += std::to_string(l.first) + "|";
  }

  std::string log_start = "LOG START: " + logger_id_  + ", " + to_string(now) + ", " + loggers_str
                          + (log_file_name.size() > 0 ?  ", fn: " + log_file_name : "");
  if(star_header)
  {
    CNR_INFO_ONLY_FILE((*this), "\n ==================================\n\n"
                       << "== " << log_start << "\n\n == Ready to Log!");
  }
  else
  {
    CNR_INFO_ONLY_FILE((*this), log_start);
  }

  // =======================================================================================
  // Default Throttle Time
  //
  // =======================================================================================
  if(!extract(default_throttle_time_, _path, "default_throttle_time", -1.0))
  {
    std::cerr << logger_id_ << ": Paremeter missing: path='"<<path<<"', key='"<<"default_throttle_time"
                << ", default superimposed: "<< -1.0 <<std::endl;
  }

  initialized_ = true;
  return initialized_;
}

TraceLogger& TraceLogger::operator=(const TraceLogger& rhs)
{
  if(!this->init(rhs.logger_id_, rhs.path_, false, rhs.default_values_))
  {
    std::cerr << rhs.logger_id_ << ": ERROR - THE LOGGER INITIALIZATION FAILED." << std::endl;
  }
  return *this;
}

TraceLogger::~TraceLogger()
{
  for(auto & l : loggers_)
  {
    l.second->removeAllAppenders();
  }
}

std::string to_string(const TraceLogger& logger)
{
  std::string ret;
  ret +="ID:" + logger.logger_id_;
  ret +="\nFILE PATH: " + logger.path_;
  ret +="\nDEFAULT VALUES: " + std::to_string(logger.default_values_);
  ret +="\nINITIALIZED: " + std::to_string(logger.initialized_);
  ret +="\nTHROTTLE TIME: " + std::to_string(logger.default_throttle_time_);
  ret +="\nLEVELS: [ ";
  for(const auto & l: logger.levels_)
    ret+= (l.first == TraceLogger::FILE_STREAM          ) ? "FILE_STREAM "
        : (l.first == TraceLogger::CONSOLE_STREAM       ) ? "CONSOLE_STREAM "
        : "SYNC_FILE_AND_CONSOLE ";
  ret += "]";
  ret +="\nLOGGERS: [ ";
  for(const auto & l: logger.levels_)
    ret+= (l.first == TraceLogger::FILE_STREAM          ) ? "FILE_STREAM "
        : (l.first == TraceLogger::CONSOLE_STREAM       ) ? "CONSOLE_STREAM "
        : "SYNC_FILE_AND_CONSOLE ";
  ret += "]";
  return ret;
}

std::ostream& operator<<(std::ostream& out, const TraceLogger& logger)
{
  out << to_string(logger);
  return out;
}

}  // namespace cnr_logger
