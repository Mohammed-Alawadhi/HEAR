#include "HEAR_control/PID_Block.hpp"

namespace HEAR{


PID_Block::PID_Block(float kp, float ki, float kd): _kp(kp), _ki(ki), _kd(kd), Block(BLOCK_ID::PID){
    e = createInputPort<float>(IP::ERROR, TYPE::Float, "ERROR");
    u = createOutputPort<float>(OP::OUTPUT, TYPE::Float, "OUTPUT");
}

void PID_Block::process(){
    float error;
    e->read(error);
    e_sum += error*_dt;
    float output = _kp*error + _kd*(error-_prev)/_dt + _ki*e_sum;
    _prev = error;
    u->write(output);
}

void PID_Block::reset(){
    std::cout <<"reset function called from PID block\n";
    _prev = 0;
    e_sum = 0;
}

}