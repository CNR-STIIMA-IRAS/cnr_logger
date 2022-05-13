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

#include <cstdlib>
#include <boost/filesystem.hpp>
#include <log4cxx/helpers/transcoder.h>

#define MAX_PATH 1024

std::string get_homedir(void)
{
    std::string homedir;

#if defined(_WIN32) || defined(_WIN64)
    homedir = std::string(std::getenv("HOMEDRIVE")) + std::string(getenv("HOMEPATH"));
#else
    homedir = std::getenv("HOME");
#endif
    return homedir;
}


bool exists_test (const std::string& name, const bool is_a_file = true) 
{
  boost::filesystem::path p(name);
  if(!boost::filesystem::exists(p)) // does path p actually exist?
  {
    return false;
  }
  
  if(is_a_file && !boost::filesystem::is_regular_file(p))  // is path p a regular file?
  {
      return false;
  }

  if(!is_a_file && !boost::filesystem::is_directory(p))  // is path p a directory file?
  {
      return false;
  }

  return true;
  // struct stat buffer;   
  // return (stat (name.c_str(), &buffer) == 0); 
}


bool mkLogDir(std::string& dir)
{
  dir = get_homedir() + "/.ros/log";
  if(!exists_test (dir, false))
  {
    boost::filesystem::path p(dir);
	  if(!boost::filesystem::create_directories(p)) 
    {
		  return false;
	  }
  }
 
  return true;
}


#if !defined(_WIN32) && !defined(_WIN64)

namespace log4cxx
{


class LOG4CXX_EXPORT ColorPatternLayout : public log4cxx::PatternLayout
{
public:
  DECLARE_LOG4CXX_OBJECT(ColorPatternLayout)
  BEGIN_LOG4CXX_CAST_MAP()
  LOG4CXX_CAST_ENTRY(ColorPatternLayout)
  LOG4CXX_CAST_ENTRY_CHAIN(Layout)
  END_LOG4CXX_CAST_MAP()

  ColorPatternLayout() : log4cxx::PatternLayout() {}
  explicit ColorPatternLayout(const log4cxx::LogString &s)
    : log4cxx::PatternLayout(s)
  {

  }
  virtual void format(log4cxx::LogString &output,
                      const log4cxx::spi::LoggingEventPtr &event,
                      log4cxx::helpers::Pool &pool) const override
  {
    try
    {
      log4cxx::LogString tmp;
      if (event)
      {
        log4cxx::PatternLayout::format(tmp, event, pool);
        log4cxx::LevelPtr lvl = event->getLevel();
        std::string color ="";
        switch (lvl->toInt())
        {
        case log4cxx::Level::FATAL_INT:
          color = "\u001b[0;41m";
          break;
        case log4cxx::Level::ERROR_INT:
          color = "\u001b[0;31m";  // red FG
          break;
        case log4cxx::Level::WARN_INT:
          color = "\u001b[0;33m";  // Yellow FG
          break;
        case log4cxx::Level::INFO_INT:
          color = "\u001b[1m";     // Bright
          break;
        case log4cxx::Level::DEBUG_INT:
          color = "\u001b[1;32m";  // Green FG
          break;
        case log4cxx::Level::TRACE_INT:
          color = "\u001b[0;34m";  // Black FG
          break;
        default:
          break;
        }
        if(color.length()>0)
        {
          log4cxx::LogString _color;
          log4cxx::helpers::Transcoder::decode( color, _color);
          output.append(_color);  // red BG
        }
      }
      output.append(tmp);
      
      std::string color = "\u001b[m"; 
      log4cxx::LogString _color;
      log4cxx::helpers::Transcoder::decode( color, _color);
      output.append(_color);
    }
    catch (std::exception& e)
    {
      std::cerr << __PRETTY_FUNCTION__<<":" << __LINE__<<": Exception: "<< e.what() << std::endl;
    }
  }
};

LOG4CXX_PTR_DEF(ColorPatternLayout);

}  // namespace log4cxx

using log4cxx::ColorPatternLayout;
IMPLEMENT_LOG4CXX_OBJECT(ColorPatternLayout);

#endif

namespace cnr_logger
{

/**
 * @brief Utility to print nicely the time.
 * @param now Time to translate in string.
 */
#if defined(ROS_NOT_AVAILABLE) || !defined(FORCE_ROS_TIME_USE)
std::string to_string(const time_t& now)
#else
std::string to_string(const ros::Time& now)
#endif
{
  std::string ret;
#if defined(ROS_NOT_AVAILABLE) || !defined(FORCE_ROS_TIME_USE)
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
              const YAML::Node* path,
                const std::string& leaf,
                    const T& default_val)
#else
bool extract(T& val,
              const std::string* path,
                const std::string& leaf,
                  const T& default_val)
#endif
{
  bool ret = false;
#if defined(ROS_NOT_AVAILABLE)
  ret = path != nullptr ? (*path)[leaf] : false;
#else
  ret = path != nullptr ? ros::param::get(*path + "/" + leaf, val) : false;
#endif
  if(!ret)
  {
    val = default_val;
  }
#if defined(ROS_NOT_AVAILABLE)
  else
  {
    val = (*path)[leaf].as<T>();
  }
#endif
  return ret;
}

#if defined(ROS_NOT_AVAILABLE)
bool extractVector(std::vector<std::string>& val,
              const YAML::Node* path,
                const std::string& leaf,
                    const std::vector<std::string>& default_val)
#else
bool extractVector(std::vector<std::string>& val,
                      const std::string* path,
                        const std::string& leaf,
                          const std::vector<std::string>& default_val)
#endif
{
  bool ret = false;
#if defined(ROS_NOT_AVAILABLE)
  ret = (path != nullptr) ? (*path)[leaf] : false;
#else
  ret = (path != nullptr) ? ros::param::get(*path + "/" + leaf, val) : false;
#endif
  if(!ret && default_val.size())
  {
    val.resize(default_val.size());
    std::copy(default_val.begin(), default_val.end(), val.begin());
  }
#if defined(ROS_NOT_AVAILABLE)
  else if(ret)
  {
    for(YAML::const_iterator it=(*path)[leaf].begin();it!=(*path)[leaf].end();++it)
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
                         const bool star_header, const bool default_values, std::string* what)
  : logger_id_(""), path_(""), default_values_(false), initialized_(false) // TraceLogger() Enrico 03/12/2021 the compiler can't find the TraceLogger() constructor
{
  std::string err = "[" + logger_id + "] Error in creating the TraceLogger.\n"
      +  std::string("INPUT logger_id      : ") + logger_id + "\n"
      +  std::string("INPUT path           : ") + path + "\n"
      +  std::string("INPUT star_header    : ") + std::to_string(star_header) + "\n"
      +  std::string("INPUT default_values : ") + std::to_string(default_values) + "\n";

  try
  {
    if(init(logger_id, path, star_header, default_values, what))
    {
      return;
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "[" << logger_id << "] Exception: " << e.what() << std::endl;
  }
  catch (...)
  {
    std::cerr << "[" << logger_id << "] Unhandled Exception" << std::endl;
  }
  std::cerr << err << std::endl;
  throw std::runtime_error((std::string(__PRETTY_FUNCTION__) + "Error in init the logger. Abort.").c_str());
}


bool TraceLogger::check(const std::string& path)
{
  bool res = true;
#if defined(ROS_NOT_AVAILABLE)
  if(!exists_test(path))
  {
    res = false;
  }
  else
  {
    YAML::Node config = YAML::LoadFile(path);

    res &= (config["appenders"] && config["appenders"].IsSequence());
    res &= (config["levels"]?config["levels"].IsSequence():true);
    res &= (config["pattern_layout"]?config["pattern_layout"].IsScalar():true);
    res &= (config["file_name"]?config["file_name"].IsScalar():true);
    res &= (config["append_date_to_file_name"]?config["append_date_to_file_name"].IsScalar():true);
    res &= (config["append_to_file"]?config["append_to_file"].IsScalar():true);
  }
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

bool TraceLogger::init_logger(const std::string& logger_id, const std::string& path,
                          const bool star_header, const bool default_values, std::string* what)
{
  return this->init(logger_id, path, star_header, default_values, what);
}

bool TraceLogger::init(const std::string& logger_id, const std::string& path,
                          const bool star_header, const bool default_values, std::string* what)
{
  if(initialized_)
  {
    if(what)
      *what = "Logger ID: " + logger_id_ + ", Logger already initialized.";
    return false;
  }

  logger_id_ = logger_id;
  path_ = path;
  default_values_ = default_values;

  if((!default_values) && (!check(path)))
  {
    if(what)
    {
      *what =  "Logger ID:" + logger_id +  ", Error in initialization. "
              +  "  [IN: default_values=" + std::to_string(default_values) + ", path=" + path + " ]";
    }
    return false;
  }

  // =======================================================================================
  // Extract the info to constructs the logger
  //
  // =======================================================================================
  std::vector<std::string> appenders, levels;
#if defined(ROS_NOT_AVAILABLE)
  YAML::Node* _path = nullptr;
  if(exists_test(path))
  {
     _path = new YAML::Node();
    *_path = YAML::LoadFile(path);
  } 
  auto now = std::time(0); 
#else
  const std::string* _path = &path;
  #if defined(FORCE_ROS_TIME_USE)
    auto now = ros::Time::now();
  #else
    auto now = std::time(0);
  #endif
#endif



  std::vector<std::string> empty;
  std::vector<std::string> warnings;
  if(!extractVector(appenders, _path, "appenders", empty))
  {
    warnings.push_back("Paremeter missing: path='"+path+"', key='appenders'");
  }
  
  if(!extractVector(levels, _path, "levels", empty))
  {
    warnings.push_back("Paremeter missing: path='"+path+"', key='levels'");
  }

  if(appenders.size() != levels.size())
  {
    warnings.push_back("Size of appenders and levels mismatch! Default DEBUG level for all the appenders");
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
    warnings.push_back("Paremeter missing: path='"+path+"', key='pattern_layout'");
  }
  log4cxx::LogString _pattern_layout;
  log4cxx::helpers::Transcoder::decode( pattern_layout, _pattern_layout);
  

#if !defined(_WIN32) && !defined(_WIN64)
  log4cxx::ColorPatternLayoutPtr layout = new log4cxx::ColorPatternLayout(_pattern_layout);
#else
  log4cxx::PatternLayoutPtr layout = new log4cxx::PatternLayout(_pattern_layout);
#endif
  // =======================================================================================



  // =======================================================================================
  // Configure appenders and loggers
  //
  // =======================================================================================
  std::string log_file_name;
  std::string root_path; 
  if(!mkLogDir(root_path))
  {
    if(what)
      *what = "Logger ID: " + logger_id_ + ", Impossible to create/access the log directory";
    return false;
  }
  std::string default_log_file_name = log_file_name = root_path + "/" + logger_id_;
  if(logFile() || logSyncFileAndScreen())
  {
    if(!extract(log_file_name, _path, "file_name", default_log_file_name))
    {
      warnings.push_back("Paremeter missing: path='" + path + "', key='file_name'");
      warnings.push_back("Parameter superimposed: log file name= '" + default_log_file_name +"'");
    }

    bool append_date_to_file_name = false;
    bool default_append_date_to_file_name = false;
    if(!extract(append_date_to_file_name, _path, "append_date_to_file_name", default_append_date_to_file_name))
    {
      warnings.push_back("Paremeter missing: path='" + path + "', key='append_date_to_file_name'");
      warnings.push_back("Parameter superimposed: append date to file= '" + std::to_string(append_date_to_file_name) +"'");
    }

    log_file_name +=  append_date_to_file_name  ? ("." + to_string(now) + ".log") : ".log";

    bool append_to_file = false;
    bool default_append_to_file = true;
    if(!extract(append_to_file, _path, "append_to_file", default_append_to_file))
    {
      warnings.push_back("Paremeter missing: path='" + path + "', key='append_to_file'");
      warnings.push_back("Parameter superimposed: append data to file= '" + std::to_string(default_append_to_file) +"'");
    }


    log4cxx::LogString _log_file_name;
    log4cxx::helpers::Transcoder::decode(log_file_name,_log_file_name);
    log4cxx::RollingFileAppenderPtr appender = new log4cxx::RollingFileAppender(layout, _log_file_name, append_to_file);
    loggers_[ logFile() ? FILE_STREAM : SYNC_FILE_AND_CONSOLE ]->addAppender(appender);

    appender->setMaximumFileSize(100 * 1024 * 1024);
    appender->setMaxBackupIndex(10);
    log4cxx::helpers::Pool pool;
    appender->activateOptions(pool);
  }

  if(logScreen() || logSyncFileAndScreen())
  {
    log4cxx::LogString _logger_id_;
    log4cxx::helpers::Transcoder::decode(logger_id_,_logger_id_);
    log4cxx::ConsoleAppenderPtr appender = new log4cxx::ConsoleAppender(layout, _logger_id_);
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
    warnings.push_back("Paremeter missing: path='" + path + "', key='default_throttle_time'");
    warnings.push_back("Parameter superimposed: default throttle time= '" + std::to_string(default_throttle_time_) +"'");
  }
  if(what)
  {
    *what = "Logger Creation has active warnings [Logger ID: " + logger_id_ + "]\n";
    for(size_t i=0;i<warnings.size();i++)
    {
      *what += std::to_string(i+1) + "#1 " + warnings.at(i) + "\n";
    }
  }

  initialized_ = true;
  return initialized_;
}

TraceLogger& TraceLogger::operator=(const TraceLogger& rhs)
{
  std::string what;
  if(!this->init(rhs.logger_id_, rhs.path_, false, rhs.default_values_, &what))
  {
    std::cerr << rhs.logger_id_ << ": ERROR - THE LOGGER INITIALIZATION FAILED." << std::endl;
    std::cerr << what << std::endl;
    throw std::runtime_error((std::string(__PRETTY_FUNCTION__) + "Error in initializing the logger()").c_str());
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
