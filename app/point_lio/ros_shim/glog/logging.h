#pragma once

#include <cstdlib>
#include <iostream>

namespace google {
inline void InitGoogleLogging(const char*) {}
}

namespace app::point_lio::shim {

class LogStream {
public:
    explicit LogStream(std::ostream& stream) : stream_(stream) {}

    template <typename T>
    LogStream& operator<<(const T& value) {
        stream_ << value;
        return *this;
    }

private:
    std::ostream& stream_;
};

class NullLogStream {
public:
    template <typename T>
    NullLogStream& operator<<(const T&) { return *this; }
};

} // namespace app::point_lio::shim

#define INFO std::cout
#define WARNING std::cerr
#define ERROR std::cerr
#define FATAL std::cerr
#define LOG(level) app::point_lio::shim::LogStream(level)
#define VLOG(level) app::point_lio::shim::NullLogStream()
#define CHECK(condition) if (!(condition)) std::abort(); else app::point_lio::shim::NullLogStream()
#define DCHECK(condition) app::point_lio::shim::NullLogStream()
