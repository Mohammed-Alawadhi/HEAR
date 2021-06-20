#ifndef TIMER_HPP
#define TIMER_HPP

namespace HEAR{

#include <chrono>

class Timer {
    private:
    std::chrono::time_point<std::chrono::system_clock> _start;
    public:
    void tick();
    int tockMicroSeconds();
    int tockMilliSeconds();
};

void Timer::tick() {
    _start=std::chrono::system_clock::now();
}
int Timer::tockMicroSeconds() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - _start).count();
}
int Timer::tockMilliSeconds() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _start).count();
}

}

#endif