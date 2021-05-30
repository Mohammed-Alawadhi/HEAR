#ifndef PID_BLOCK_HPP
#define PID_BLOCK_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include <iostream>

namespace HEAR{

class PID_Block : public Block{
private:
    InputPort<float>* e;
    OutputPort<float>* u;
    float _prev=0, _kp, _ki, _kd, e_sum=0;

public:
    enum IP{ERROR};
    enum OP{OUTPUT};
    PID_Block(float kp, float ki, float kd);
    ~PID_Block(){ delete e, u;}
    void process();

    void reset();
};

}

#endif