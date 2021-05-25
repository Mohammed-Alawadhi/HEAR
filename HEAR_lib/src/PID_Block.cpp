#include "HEAR_control/PID_Block.hpp"

namespace HEAR{


PID_Block::PID_Block(float dt): _dt(dt), Block(BLOCK_ID::PID){
    createPorts(1, 1);
    e = createInputPort<float>(IP::ERROR, TYPE::Float, "ERROR");
    u = createOutputPort<float>(OP::OUTPUT, TYPE::Float, "OUTPUT");
}

PID_Block::~PID_Block(){}

void PID_Block::process(){
    float error;
    e->read(error);
    e_sum += error;
    float output = _kp*error + _kd*(error-_prev)/_dt + _ki*e_sum;
    _prev = error;
    u->write(output);
}

}