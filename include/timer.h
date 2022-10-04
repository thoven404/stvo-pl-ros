#pragma once

// STL
#include <chrono>

namespace StVO {

class Timer {
public:

    static constexpr double SECONDS = 1e-9;
    static constexpr double MILLISECONDS = 1e-6;
    static constexpr double NANOSECONDS = 1.0;

    Timer(double scale = MILLISECONDS);
    virtual ~Timer();

    void start();

    double stop();

private:

    std::chrono::high_resolution_clock::time_point start_t;
    bool started;
    double scale;
};

} // namespace StVO