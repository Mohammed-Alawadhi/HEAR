#pragma once

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include <map>

namespace HEAR{

class PID_Block : public Block{
private:
    InputPort<float>* e;
    OutputPort<float>* u;
    float _prev, _dt, _kp, _ki, _kd, e_sum;

public:
    enum IP{ERROR};
    enum OP{OUTPUT};
    PID_Block(float);
    ~PID_Block();
    void init();
    void process();
};

}