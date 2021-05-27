#pragma once

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{

class PID_Block : public Block{
private:
    InputPort* e;
    OutputPort* u;
    FloatMsg port_data;
    float _prev, _dt, _kp, _ki, _kd, e_sum;

public:
    enum IP{ERROR};
    enum OP{OUTPUT};
    PID_Block(float dt);
    ~PID_Block();
    void process();
};

}