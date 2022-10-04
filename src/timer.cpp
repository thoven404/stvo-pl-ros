#include "timer.h"

//STL
#include <stdexcept>
#include <chrono>

namespace StVO {

Timer::Timer(double scale) : started(false), scale(scale) { }
Timer::~Timer() { }

void Timer::start() {

    started = true;
    start_t = std::chrono::high_resolution_clock::now();
}

double Timer::stop() {

    std::chrono::high_resolution_clock::time_point end_t = std::chrono::high_resolution_clock::now();

    if (!started)
        throw std::logic_error("[Timer] Stop called without previous start");

    started = false;
    std::chrono::duration<double, std::nano> elapsed_ns = end_t - start_t;
    return elapsed_ns.count()*scale;
}

} // namespace StVO