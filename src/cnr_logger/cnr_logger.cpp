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
#include <cctype>
#include <array>

#include <log4cxx/spi/location/locationinfo.h>
#include <log4cxx/helpers/exception.h>
#include <log4cxx/helpers/transcoder.h>

#include <log4cxx/rollingfileappender.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/patternlayout.h>

//
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

#if !defined(Log4cxx_MAJOR_VERSION) 
  #error The Version of Log4cxx are not available. Add -DLog4cxx_MAJOR_VERSION to compile options
#endif


//========================================================
std::tm localtime_xp(const std::time_t& timer)
{
    std::tm bt {};
#if defined(_MSC_VER)
    localtime_s(&bt, &timer);
#else
  localtime_r(&timer, &bt);
#endif
    return bt;
}

// default = "YYYY-MM-DD HH:MM:SS"
std::string time_stamp(const std::time_t& timer, const std::string& fmt = "%F %T")
{
  auto bt = localtime_xp(timer);
  std::array<char, 128> buf;
  size_t error = std::strftime(buf.data(), sizeof(buf), fmt.c_str(), &bt);
  if(!error)
  {
    std::cerr << __PRETTY_FUNCTION__ << ":" << __LINE__ << ": The buffer memory is not enough ..." << std::endl;
  }
  return buf.data();
}

std::string get_env(const char *name)
{
  
  std::string ret;
  #if defined(_MSC_VER)
    size_t size = 0;
    char result[1024];
    
    getenv_s( &size, NULL, 0, name);
   if (size == 0)
   {
      printf("%s doesn't exist!\n",name);
      return "";
   }
   // Get the value of the LIB environment variable.
    errno_t err = getenv_s(&size, result, size, name);
    
    ret = result;
  #else
    const char* result = std::getenv(name);
    ret = result;
  #endif

  return ret;
}


std::string get_homedir(void)
{
    std::string homedir;

#if defined(_WIN32) || defined(_WIN64)
    
    homedir = std::string(get_env("HOMEDRIVE")) + std::string(get_env("HOMEPATH"));
#else
    homedir = std::getenv("HOME");
#endif
    return homedir;
}


bool exists_test (const std::string& name, const bool is_a_file = true) 
{
  boost::filesystem::path p(name);

  return boost::filesystem::exists(p) 
          && (is_a_file ? boost::filesystem::is_regular_file(p) 
              : boost::filesystem::is_directory(p));
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
//========================================================




namespace log4cxx
{

class ColorPatternLayout : public log4cxx::PatternLayout
{
public:
    DECLARE_LOG4CXX_OBJECT(ColorPatternLayout)
    BEGIN_LOG4CXX_CAST_MAP()
        LOG4CXX_CAST_ENTRY(ColorPatternLayout)
        LOG4CXX_CAST_ENTRY_CHAIN(Layout)
    END_LOG4CXX_CAST_MAP()

    ColorPatternLayout();
    explicit ColorPatternLayout(const log4cxx::LogString &s);
    void format(log4cxx::LogString &output,
                  const log4cxx::spi::LoggingEventPtr &event,
                    log4cxx::helpers::Pool &pool) const override;
};
LOG4CXX_PTR_DEF(ColorPatternLayout);
}

using namespace log4cxx;
IMPLEMENT_LOG4CXX_OBJECT(ColorPatternLayout);
ColorPatternLayout::ColorPatternLayout() : log4cxx::PatternLayout(){}

ColorPatternLayout::ColorPatternLayout(const LogString &s): log4cxx::PatternLayout(s){}

void ColorPatternLayout::format(LogString &output, const spi::LoggingEventPtr &event, helpers::Pool &pool) const
{
    log4cxx::LogString tmp;
    log4cxx::PatternLayout::format(tmp,event,pool);
    log4cxx::LevelPtr lvl = event->getLevel();
    log4cxx::LogString _color;
    switch (lvl->toInt())
    {
    case log4cxx::Level::FATAL_INT:
        log4cxx::helpers::Transcoder::decode("\u001b[0;41m", _color);
        break;
    case log4cxx::Level::ERROR_INT:
        log4cxx::helpers::Transcoder::decode("\u001b[0;31m", _color);
        break;
    case log4cxx::Level::WARN_INT:
        log4cxx::helpers::Transcoder::decode("\u001b[0;33m", _color);
        break;
    case log4cxx::Level::INFO_INT:
        log4cxx::helpers::Transcoder::decode("\u001b[1m", _color);
        break;
    case log4cxx::Level::DEBUG_INT:
        log4cxx::helpers::Transcoder::decode("\u001b[2;32m", _color);
        break;
    case log4cxx::Level::TRACE_INT:
        log4cxx::helpers::Transcoder::decode("\u001b[0;30m", _color);
        break;
    default:
        break;
    }
    if(_color.size()>0)
    {
      output.append(_color); 
    }
    output.append(tmp);

    log4cxx::helpers::Transcoder::decode("\u001b[m", _color);
    output.append(_color);
}

#if defined(ROS_NOT_AVAILABLE)
  using resource_t = YAML::Node;
#else
  using resource_t = std::string;
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
  ret = time_stamp(now);
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
bool extract(T& val,
              const resource_t* res,
                const std::string& leaf,
                    const T& default_val)
{
  bool ret = false;
#if defined(ROS_NOT_AVAILABLE)
  ret = (res != nullptr) ? bool( (*res)[leaf] ): false;
#else
  ret = (res != nullptr) ? ros::param::get(*res + "/" + leaf, val) : false;
#endif
  if(!ret)
  {
    val = default_val;
  }
#if defined(ROS_NOT_AVAILABLE)
  else
  {
    val = (*res)[leaf].as<T>();
  }
#endif
  return ret;
}

bool extractVector(std::vector<std::string>& val,
                      const resource_t* res,
                        const std::string& leaf,
                          const std::vector<std::string>& default_val)
{
  bool ret = false;
#if defined(ROS_NOT_AVAILABLE)
  ret = (res != nullptr) ? bool( (*res)[leaf] ) : false;
#else
  ret = (res != nullptr) ? ros::param::get(*res + "/" + leaf, val) : false;
#endif
  if(!ret && default_val.size())
  {
    val.resize(default_val.size());
    std::copy(default_val.begin(), default_val.end(), val.begin());
  }
#if defined(ROS_NOT_AVAILABLE)
  else if(ret)
  {
    for(YAML::const_iterator it=(*res)[leaf].begin();it!=(*res)[leaf].end();++it)
    {
      val.emplace_back(it->as<std::string>());
    }
   }
#endif
  return ret;
}

void if_error_fill(std::string* what, const std::string& msg, const bool append = false)
{
  if(what)
  {
    if(append)
    {
      *what += msg;
    }
    else
    {
      *what = msg;
    }
  }
}

std::string appender2string(const TraceLogger::AppenderType& t)
{
  std::string ret ="";
  switch(t)
  {
    case TraceLogger::AppenderType::FILE_STREAM: ret = "FILE_STREAM"; break;
    case TraceLogger::AppenderType::CONSOLE_STREAM: ret = "CONSOLE_STREAM"; break;
    case TraceLogger::AppenderType::SYNC_FILE_AND_CONSOLE: ret = "SYNC_FILE_AND_CONSOLE"; break;
  }
  return ret;
}

std::string level2string(const TraceLogger::Level& t)
{
  std::string ret ="";
  switch(t)
  {
    case TraceLogger::Level::FATAL: ret = "FATAL"; break;
    case TraceLogger::Level::ERROR: ret = "ERROR"; break;
    case TraceLogger::Level::WARN: ret = "WARN"; break;
    case TraceLogger::Level::INFO: ret = "INFO"; break;
    case TraceLogger::Level::DEBUG: ret = "DEBUG"; break;
    case TraceLogger::Level::TRACE: ret = "TRACE"; break;
  }
  return ret;
}

TraceLogger::Level string2level(const std::string& what)
{
  TraceLogger::Level ret;
  if(what == "FATAL") 
  {
    ret = TraceLogger::Level::FATAL;
  }
  else if(what == "ERROR")
  {
    ret = TraceLogger::Level::ERROR;
  }
  else if(what == "WARN")
  {
    ret = TraceLogger::Level::WARN;
  }
  else if(what == "INFO")
  {
    ret = TraceLogger::Level::INFO;
  }
  else if(what == "DEBUG")
  {
    ret = TraceLogger::Level::DEBUG;
  }
  else
  {
    ret = TraceLogger::Level::TRACE;
  }
  return ret;
}


void setLoggers(const std::string& logger_id, 
                  const std::vector<std::string>& appenders_data,
                    const std::vector<std::string>& levels_data,
                      std::map<TraceLogger::AppenderType, log4cxx::LoggerPtr>& loggers,
                        std::map<TraceLogger::AppenderType, TraceLogger::Level>& levels, 
                          TraceLogger::Level& max_level)
{
  auto it_file     = std::find(appenders_data.begin(), appenders_data.end(), "file");
  auto it_screen   = std::find(appenders_data.begin(), appenders_data.end(), "screen");
  int  idx_file    = (it_file   != appenders_data.end()) ? (int)std::distance(appenders_data.begin(), it_file) : -1;
  int  idx_screen  = (it_screen != appenders_data.end()) ? (int)std::distance(appenders_data.begin(), it_screen) : -1;

  if(((idx_file >= 0) && (idx_screen >= 0)) && (levels_data.at(idx_file) == levels_data.at(idx_screen)))   // 1 logger and 2 appenders
  {
    loggers  [ TraceLogger::AppenderType::SYNC_FILE_AND_CONSOLE ] = log4cxx::Logger::getLogger(logger_id);
    levels   [ TraceLogger::AppenderType::SYNC_FILE_AND_CONSOLE ] = string2level(levels_data[idx_file]);
  }
  else
  {
    if(idx_file >= 0)
    {
      loggers[TraceLogger::AppenderType::FILE_STREAM] = log4cxx::Logger::getLogger(logger_id + "_f");
      levels [TraceLogger::AppenderType::FILE_STREAM] = string2level(levels_data[idx_file]);
    }
    if(idx_screen >= 0)
    {
      loggers[TraceLogger::AppenderType::CONSOLE_STREAM] = log4cxx::Logger::getLogger(logger_id);
      levels [TraceLogger::AppenderType::CONSOLE_STREAM] = string2level(levels_data.at(idx_screen));
    }
  }

  max_level = TraceLogger::Level::FATAL;
  for(auto const & level : levels)
  {
    switch(level.second)
    {
      case TraceLogger::Level::FATAL: loggers[level.first]->setLevel(log4cxx::Level::getFatal()); break;
      case TraceLogger::Level::ERROR: loggers[level.first]->setLevel(log4cxx::Level::getError()); break;
      case TraceLogger::Level::WARN : loggers[level.first]->setLevel(log4cxx::Level::getWarn() ); break;
      case TraceLogger::Level::INFO : loggers[level.first]->setLevel(log4cxx::Level::getInfo() ); break;
      case TraceLogger::Level::DEBUG: loggers[level.first]->setLevel(log4cxx::Level::getDebug()); break;
      case TraceLogger::Level::TRACE: loggers[level.first]->setLevel(log4cxx::Level::getTrace()); break;
    }
    max_level = (level.second >= max_level) ? level.second : max_level;
  }
}



void extractLayout(const std::string& ns, 
                    const resource_t* res,
                      log4cxx::PatternLayoutPtr& layout,
                        std::vector<std::string>& warnings)
{
  #if defined(__clang__) || defined(_MSC_VER)
    const std::string default_pattern_layout = "[%5p][%d{HH:mm:ss,SSS}][%F:%L][%c] %m%n"; 
  #else
    const std::string default_pattern_layout = "[%5p][%d{HH:mm:ss,SSS}][%M:%L][%c] %m%n";
  #endif
  
  std::string pattern_layout;
  if(!extract(pattern_layout, res, "pattern_layout", default_pattern_layout))
  {
    warnings.emplace_back("Parameter missing '"+ns+"/pattern_layout'");
  }

  log4cxx::LogString _pattern_layout;
  log4cxx::helpers::Transcoder::decode(pattern_layout, _pattern_layout);
  
#if !defined(_WIN32) && !defined(_WIN64)
  #if (Log4cxx_MAJOR_VERSION==0) && (Log4cxx_MINOR_VERSION > 10)
    layout.reset( new log4cxx::ColorPatternLayout(_pattern_layout) );
  #else
    layout = new log4cxx::ColorPatternLayout(_pattern_layout);
  #endif
#else
  layout = new log4cxx::PatternLayout(_pattern_layout);
#endif
}


bool getFileName(const std::string& ns, 
                    const resource_t* res,
                      const std::string& logger_id,
                        std::string& log_file_name, 
                          bool& append_to_file, 
                            std::vector<std::string>& warnings)
{
  
  std::string root_path; 
  if(!mkLogDir(root_path))
  {
    warnings.emplace_back("Logger ID: " + logger_id + ", Impossible to create/access the log directory");
    return false;
  }
  std::string default_log_file_name = log_file_name = root_path + "/" + logger_id;

  
  if(!extract(log_file_name, res, "file_name", default_log_file_name))
  {
    warnings.emplace_back("Parameter missing '"+ns+"/file_name'");
    warnings.emplace_back("Parameter superimposed '"+ns+"/file_name= '" + default_log_file_name +"'");
  }

  
  bool append_date_to_file_name = false;
  bool default_append_date_to_file_name = false;
  if(!extract(append_date_to_file_name, res, "append_date_to_file_name", default_append_date_to_file_name))
  {
    warnings.emplace_back("Parameter missing '"+ns+"/append_date_to_file_name'");
    warnings.emplace_back("Parameter superimposed '"+ns+"/append_date_to_file_name= '" + std::to_string(append_date_to_file_name) +"'");
  }

  
  #if defined(ROS_NOT_AVAILABLE) || !defined(FORCE_ROS_TIME_USE)
    auto now = std::time(nullptr); 
  #else
      auto now = ros::Time::now();
  #endif

  log_file_name +=  append_date_to_file_name  ? ("." + to_string(now) + ".log") : ".log";

  
  bool default_append_to_file = true;
  if(!extract(append_to_file, res, "append_to_file", default_append_to_file))
  {
    warnings.emplace_back("Parameter missing '"+ns+"/append_to_file'");
    warnings.emplace_back("Parameter superimposed '"+ns+"/append_to_file= '" + std::to_string(default_append_to_file) +"'");
  }
  return true;
}

std::string getLoggerStartString(const std::string& logger_id, 
                                  const std::string& log_file_name,
                                    const std::map<TraceLogger::AppenderType, log4cxx::LoggerPtr>& loggers,
                                      const std::map<TraceLogger::AppenderType, TraceLogger::Level>& levels)
{
  std::string loggers_str = "n. app.: " + std::to_string(static_cast<int>(loggers.size())) + " ";
  for(auto const & l : loggers)
  {
    loggers_str += appender2string(l.first) + "|";
  }

  loggers_str += (levels.empty() ?  "" : " l: ");
  for(auto const & l : levels)
  {
    loggers_str += level2string(l.second) + "|";
  }

  #if defined(ROS_NOT_AVAILABLE) || !defined(FORCE_ROS_TIME_USE)
    auto now = std::time(nullptr); 
  #else
      auto now = ros::Time::now();
  #endif

  return "LOG START: " + logger_id  + ", " + to_string(now) + ", " + loggers_str
            + ( (log_file_name.size() > 0u) ?  ", fn: " + log_file_name : "");
}


bool configureLoggers(const std::string& logger_id,
                        const std::string& log_file_name, 
                          const bool& append_to_file,
                            const log4cxx::PatternLayoutPtr& layout, 
                              std::map<TraceLogger::AppenderType, log4cxx::LoggerPtr>& loggers,
                                std::vector<std::string>& warnings)
{
  std::string root_path; 
  if(!mkLogDir(root_path))
  {
    warnings.emplace_back("Logger ID: " + logger_id + ", Impossible to create/access the log directory");
    return false;
  }

  if(loggers.find(TraceLogger::AppenderType::FILE_STREAM) != loggers.end()
   || (loggers.find(TraceLogger::AppenderType::SYNC_FILE_AND_CONSOLE) != loggers.end()) )
  {

    log4cxx::LogString _log_file_name;
    log4cxx::helpers::Transcoder::decode(log_file_name,_log_file_name);
    log4cxx::RollingFileAppenderPtr appender( new log4cxx::RollingFileAppender(layout, _log_file_name, append_to_file) );
    if(loggers.find(TraceLogger::AppenderType::FILE_STREAM) != loggers.end())
    {
      loggers[ TraceLogger::AppenderType::FILE_STREAM ]->addAppender(appender);
    }
    else
    {
      loggers[ TraceLogger::AppenderType::SYNC_FILE_AND_CONSOLE ]->addAppender(appender);
    }

    appender->setMaximumFileSize(100 * 1024 * 1024);
    appender->setMaxBackupIndex(10);
    log4cxx::helpers::Pool pool;
    appender->activateOptions(pool);
  }

  if((loggers.find(TraceLogger::AppenderType::CONSOLE_STREAM) != loggers.end())
  || (loggers.find(TraceLogger::AppenderType::SYNC_FILE_AND_CONSOLE) != loggers.end()) )
  {
    log4cxx::LogString _logger_id_;
    log4cxx::helpers::Transcoder::decode(logger_id,_logger_id_);
    log4cxx::ConsoleAppenderPtr appender(new log4cxx::ConsoleAppender(layout, _logger_id_) );
    if(loggers.find(TraceLogger::AppenderType::CONSOLE_STREAM) != loggers.end())
    {
      loggers[TraceLogger::AppenderType::CONSOLE_STREAM]->addAppender(appender);
    }
    else
    {
      loggers[TraceLogger::AppenderType::SYNC_FILE_AND_CONSOLE]->addAppender(appender);
    }
  }
  return true;
}

void getThrottletime(const std::string& ns, 
                      const resource_t* res,
                        double& default_throttle_time, 
                          std::vector<std::string>& warnings)
{
  // =======================================================================================
  // Default Throttle Time
  //
  // =======================================================================================
  if(!extract(default_throttle_time, res, "default_throttle_time", -1.0))
  {
    warnings.emplace_back("Parameter missing '"+ns+"/default_throttle_time'");
    warnings.emplace_back("Parameter superimposed ''"+ns+"/default throttle time= '" + std::to_string(default_throttle_time) +"'");
  }
}


TraceLogger::TraceLogger(const std::string& logger_id, const std::string& path,
                         const bool star_header, const bool default_values, std::string* what)
  // : logger_id_(""), path_(""), default_values_(false), initialized_(false) // TraceLogger() Enrico 03/12/2021 the compiler can't find the TraceLogger() constructor
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
  catch (const std::exception& e)
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


bool TraceLogger::check(const std::string& path) const
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


void extractAppendersAndLevels(const std::string& ns,
                                const resource_t* res,
                                  std::vector<std::string>& appenders,
                                    std::vector<std::string>& levels,
                                      std::vector<std::string>& warnings)
{
  appenders.clear();
  levels.clear();

  std::vector<std::string> empty;
  if(!extractVector(appenders, res, "appenders", empty))
  {
    warnings.emplace_back("Parameter missing '" + ns +"/appenders'");
  }
  
  if(!extractVector(levels, res, "levels", empty))
  {
    warnings.emplace_back("Parameter missing'" +ns +"/levels'");
  }

  if(appenders.size() != levels.size())
  {
    warnings.emplace_back("'" + ns + 
      "': Size of appenders and levels mismatch! Default DEBUG level for all the appenders");
    levels.clear();
    levels.resize(appenders.size(), "DEBUG");
  }


  for(size_t i = 0; i < appenders.size(); i++)
  {
    std::for_each(appenders[i].begin(), appenders[i].end(), [](char & c)
    {
      c = static_cast<char>(std::tolower(static_cast<int>(c)));
    });
    std::for_each(levels   [i].begin(), levels   [i].end(), [](char & c)
    {
      c = static_cast<char>(std::toupper(static_cast<int>(c)));
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
}


bool TraceLogger::init(const std::string& logger_id, const std::string& path,
                          const bool star_header, const bool default_values, std::string* what)
{
  
  if(initialized_)
  {
    if_error_fill(what, "Logger ID: " + logger_id_ + ", Logger already initialized.");
    return false;
  }

  
  logger_id_      = logger_id;
  path_           = path;
  default_values_ = default_values;

  
  if((!default_values) && (!check(path)))
  {
    if_error_fill(what, "Logger ID:" + logger_id +  ", Error in initialization. "
              +  "  [IN: default_values=" + std::to_string(default_values) + ", path=" + path + " ]");
    return false;
  }

  
  // =======================================================================================
  // Extract the info to constructs the logger
  //
  // =======================================================================================
  std::vector<std::string> appenders_data;
  std::vector<std::string> levels_data;
  std::vector<std::string> warnings;

  
#if defined(ROS_NOT_AVAILABLE)
  YAML::Node* resource = nullptr;
  if(exists_test(path))
  {
     resource = new YAML::Node();
    *resource = YAML::LoadFile(path);
  } 
#else
  const std::string* resource = &path;
#endif

  
  // =======================================================================================
  extractAppendersAndLevels(path, resource, appenders_data,levels_data, warnings);

  
  // =======================================================================================
  setLoggers(logger_id_,  appenders_data, levels_data, loggers_, levels_, max_level_);
  

  
  // =======================================================================================
  // Layout
  //
  // =======================================================================================
  log4cxx::PatternLayoutPtr layout;
  extractLayout(path, resource, layout, warnings);
  // =======================================================================================

  
  // =======================================================================================
  // Default Throttle Time
  //
  // =======================================================================================
  getThrottletime(path, resource, default_throttle_time_, warnings);

  
  // =======================================================================================
  // Configure appenders and loggers
  //
  // =======================================================================================
  bool append_to_file;
  std::string log_file_name;
  bool ok = getFileName(path, resource, logger_id,log_file_name, append_to_file, warnings);
  if(ok)
  {
    ok = configureLoggers(logger_id_, log_file_name, append_to_file, layout, loggers_, warnings);
    if(ok)
    {
      std::string log_start =  getLoggerStartString(logger_id, log_file_name,loggers_,levels_);
      if(star_header)
      {
        CNR_INFO_ONLY_FILE( this, "\n ==================================\n\n" << "== " << log_start << "\n\n == Ready to Log!");
      }
      else
      {
        CNR_INFO_ONLY_FILE(this, log_start);
      }
    }
  }

  
  for(size_t i=0;i<warnings.size();i++)
  {
    if_error_fill(what, std::to_string(i+1) + "#1 " + warnings.at(i) + "\n", true);
  }

  initialized_ = ok;
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
  std::for_each(loggers_.begin(), loggers_.end(), [](auto& l)
  {
    l.second->removeAllAppenders();
  });
}

bool TraceLogger::logFile()
{
  return (loggers_.find(AppenderType::FILE_STREAM) != loggers_.end()) 
    || (loggers_.find(AppenderType::SYNC_FILE_AND_CONSOLE) != loggers_.end());
}

bool TraceLogger::logScreen()
{
  return (loggers_.find(AppenderType::CONSOLE_STREAM) != loggers_.end())
    || (loggers_.find(AppenderType::SYNC_FILE_AND_CONSOLE) != loggers_.end());
}

bool TraceLogger::logOnlyFile()
{
  return loggers_.find(AppenderType::FILE_STREAM) != loggers_.end();
}

bool TraceLogger::logOnlyScreen()
{
  return (loggers_.find(AppenderType::CONSOLE_STREAM) != loggers_.end());
}

/**
 * @brief Configuration getter.
 * @return True if the logger has an appender linked to both the console and the file, false otherwise
 */
bool TraceLogger::logSyncFileAndScreen()
{
  return (loggers_.find(AppenderType::SYNC_FILE_AND_CONSOLE) != loggers_.end());
}

const double& TraceLogger::defaultThrottleTime() const
{
  return default_throttle_time_;
}

bool TraceLogger::logFatal() const
{
  return Level::FATAL<=max_level_;
}

bool TraceLogger::logError() const
{
  return Level::ERROR<=max_level_;
}

bool TraceLogger::logWarn()  const
{
  return Level::WARN <=max_level_;
}

bool TraceLogger::logInfo()  const
{
  return Level::INFO <=max_level_;
}

bool TraceLogger::logDebug() const
{
  return Level::DEBUG<=max_level_;
}

bool TraceLogger::logTrace() const
{
  return Level::TRACE<=max_level_;
}

log4cxx::LoggerPtr TraceLogger::syncFileAndScreenLogger( )
{
  if(this->logSyncFileAndScreen())
    return loggers_[::cnr_logger::TraceLogger::AppenderType::SYNC_FILE_AND_CONSOLE];
  else
    return nullptr;
}

log4cxx::LoggerPtr TraceLogger::fileLogger( )
{
  if(this->logFile())
    return loggers_[::cnr_logger::TraceLogger::AppenderType::FILE_STREAM];
  else
    return nullptr;
}

log4cxx::LoggerPtr TraceLogger::consoleLogger( )
{
  if(this->logScreen())
    return loggers_[::cnr_logger::TraceLogger::AppenderType::CONSOLE_STREAM];
  else
    return nullptr;
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
    ret+= level2string(l.second) +" ";
  ret += "]";
  ret +="\nAPPENDER TYPES: [ ";
  for(const auto & a: logger.loggers_)
    ret+= appender2string(a.first) +" ";
  ret += "]";
  return ret;
}

std::ostream& operator<<(std::ostream& out, const TraceLogger& logger)
{
  out << to_string(logger);
  return out;
}

}  // namespace cnr_logger
