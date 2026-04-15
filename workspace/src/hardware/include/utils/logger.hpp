#pragma once

#include <functional>
#include <string>

using LogFn = std::function<void(const std::string &)>;
class Logger{
  LogFn log_info_{};
  LogFn log_warn_{};
  LogFn log_error_{};

  std::string name_ = "";

public:

  void setLogger(LogFn info, LogFn warn = {}, LogFn error = {}) {
    log_info_  = std::move(info);
    log_warn_  = std::move(warn);
    log_error_ = std::move(error);
  }

  template<size_t N, typename... A>
  void logDispatch(const LogFn &fn,
                  const char (&fmt)[N],
                  A&&... a) const {
    if (!fn) return;

    constexpr size_t BUF_SIZE = 256;

    char msg[BUF_SIZE];
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wformat-security"
    #pragma GCC diagnostic ignored "-Wformat-truncation"
    std::snprintf(msg, sizeof(msg), fmt, std::forward<A>(a)...);
    #pragma GCC diagnostic pop

    if (!name_.empty()) {
      char final[BUF_SIZE];

      // escribir prefijo (truncado si es necesario)
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wformat-security"
      #pragma GCC diagnostic ignored "-Wformat-truncation"
      int written = std::snprintf(final, sizeof(final), "[%s] ", name_.c_str());
      #pragma GCC diagnostic pop
      if (written < 0) written = 0;

      size_t offset = static_cast<size_t>(written);
      if (offset >= BUF_SIZE) offset = BUF_SIZE - 1;

      // copiar mensaje lo que quepa
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wformat-security"
      #pragma GCC diagnostic ignored "-Wformat-truncation"
      std::snprintf(final + offset,
                    BUF_SIZE - offset,
                    "%s",
                    msg);
      #pragma GCC diagnostic pop

      // asegurar terminación
      final[BUF_SIZE - 1] = '\0';

      fn(final);
    } else {
      msg[BUF_SIZE - 1] = '\0';
      fn(msg);
    }
  }

  template<size_t N, typename... Args>
  void logInfo(const char (&fmt)[N], Args&&... args) const {
    logDispatch(log_info_, fmt, std::forward<Args>(args)...);
  }

  void logInfo(const std::string& fmt) const {
    logDispatch(log_info_, "%s", fmt.c_str());
  }

  template<size_t N, typename... Args>
  void logWarn(const char (&fmt)[N], Args&&... args) const {
    logDispatch(log_warn_, fmt, std::forward<Args>(args)...);
  }

  template<size_t N, typename... Args>
  void logError(const char (&fmt)[N], Args&&... args) const {
    logDispatch(log_error_, fmt, std::forward<Args>(args)...);
  }

  void setName(std::string name){
    name_ = std::move(name);
  }

};
