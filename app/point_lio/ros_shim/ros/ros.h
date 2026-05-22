#pragma once

#include <cstdio>
#include <deque>
#include <memory>
#include <string>

namespace ros {

class Time {
public:
    Time() = default;
    explicit Time(double seconds) : seconds_(seconds) {}

    [[nodiscard]] double toSec() const { return seconds_; }
    Time fromSec(double seconds) const { return Time(seconds); }
    static Time now() { return Time(0.0); }

private:
    double seconds_ = 0.0;
};

struct Header {
    Time stamp;
    std::string frame_id;
};

class Publisher {
public:
    template <typename T>
    void publish(const T&) const {}
};

class Subscriber {};

class NodeHandle {
public:
    explicit NodeHandle(const std::string& = {}) {}

    template <typename T>
    void param(const std::string&, T& value, const T& default_value) const {
        value = default_value;
    }

    template <typename Msg, typename Callback>
    Subscriber subscribe(const std::string&, int, Callback&&) const {
        return Subscriber{};
    }

    template <typename Msg>
    Publisher advertise(const std::string&, int) const {
        return Publisher{};
    }
};

class AsyncSpinner {
public:
    explicit AsyncSpinner(int) {}
    void start() {}
};

class Rate {
public:
    explicit Rate(double) {}
    void sleep() const {}
};

inline void init(int&, char**&, const std::string&) {}
inline bool ok() { return true; }
inline void spinOnce() {}

} // namespace ros

#define ROS_WARN(...) std::fprintf(stderr, __VA_ARGS__), std::fprintf(stderr, "\n")
#define ROS_INFO(...) std::fprintf(stdout, __VA_ARGS__), std::fprintf(stdout, "\n")
#define ROS_ERROR(...) std::fprintf(stderr, __VA_ARGS__), std::fprintf(stderr, "\n")
