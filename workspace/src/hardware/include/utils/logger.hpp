#pragma once

#include <functional>
#include <string>

using LogFn = std::function<void(const std::string &)>;
class Logger{
  LogFn log_info_{};
  LogFn log_warn_{};
  LogFn log_error_{};

public:

  void setLogger(LogFn info, LogFn warn = {}, LogFn error = {}) {
    log_info_  = std::move(info);
    log_warn_  = std::move(warn);
    log_error_ = std::move(error);
  }

  template<size_t N, typename... A>
  static void logDispatch(const LogFn &fn, const char (&fmt)[N], A&&... a){
    if (!fn) return;
    
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wformat-security"
    char buf[256]; std::snprintf(buf, sizeof(buf), fmt, std::forward<A>(a)...);
    #pragma GCC diagnostic pop

    fn(buf);
  }

  template<size_t N, typename... Args>
  void logInfo(const char (&fmt)[N], Args&&... args) const {
      logDispatch(log_info_, fmt, std::forward<Args>(args)...);
  }
  template<size_t N, typename... Args>
  void logWarn(const char (&fmt)[N], Args&&... args) const {
      logDispatch(log_warn_, fmt, std::forward<Args>(args)...);
  }
  template<size_t N, typename... Args>
  void logError(const char (&fmt)[N], Args&&... args) const {
      logDispatch(log_error_, fmt, std::forward<Args>(args)...);
  }
};
