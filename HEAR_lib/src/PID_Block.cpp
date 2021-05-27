#include "HEAR_control/PID_Block.hpp"

namespace HEAR{


PID_Block::PID_Block(float dt): _dt(dt), Block(BLOCK_ID::PID, 1, 1){
    e = createInputPort(IP::ERROR, TYPE::Float, "ERROR");
    u = createOutputPort(OP::OUTPUT, new FloatMsg, "OUTPUT");
}

PID_Block::~PID_Block(){}

void PID_Block::process(){
    FloatMsg* err = (FloatMsg*)e->read();
    float error = err->data;
    e_sum += error;
    float output = _kp*error + _kd*(error-_prev)/_dt + _ki*e_sum;
    _prev = error;
    auto out = new FloatMsg();
    out->data = output;
    u->write(out);
}

}