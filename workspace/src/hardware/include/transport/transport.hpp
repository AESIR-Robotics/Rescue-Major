#pragma once

#include "utils/logger.hpp"
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <functional>
#include <string>

using stdclock  = std::chrono::steady_clock;
using deadline_t = stdclock::time_point;

class TransportInterface {
public:
    virtual ~TransportInterface() = default;

    enum class Event {
        CONNECTED,
        DISCONNECTED,
        ERROR
    };

    

    virtual bool connected() const = 0;
    virtual bool connect() = 0;
    virtual bool reconnect() = 0;

protected:
    using EventCallback = std::function<void(Event)>;
    
    void setEventCallback(EventCallback cb) {
        eventCallback_ = std::move(cb);
    }

    void clearDisconnectCallback() {
        eventCallback_ = nullptr;
    }

    void disconnect() {
        disconnectImpl();
        notifyEvent(Event::DISCONNECTED);
    }

    virtual void disconnectImpl() = 0;

    virtual bool canSend() const { return true; }
    virtual bool hasData() const { return true; }

    virtual size_t writeData(const uint8_t *data, size_t length, deadline_t deadline) = 0;
    virtual size_t readData(uint8_t *buffer, size_t length, deadline_t deadline) = 0;

    void setLogger(Logger &in_log) {
        log = in_log;
        log.setName(getName());
    }

    Logger log{};

private:

    void notifyEvent(Event ev) {
        if (eventCallback_) {
            eventCallback_(ev);
        }
    }

    virtual std::string getName() { return ""; };

    EventCallback eventCallback_; 
};