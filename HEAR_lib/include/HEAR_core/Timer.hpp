#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

namespace HEAR{

class Timer {
    private:
    std::chrono::time_point<std::chrono::system_clock> _start;
    public:
    void tick(){
        _start=std::chrono::system_clock::now();
    }
    int tockMicroSeconds(){
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - _start).count();
    }
    int tockMilliSeconds(){
        return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - _start).count();
    }
};

}

#endif