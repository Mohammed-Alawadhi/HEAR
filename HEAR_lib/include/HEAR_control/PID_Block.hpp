#ifndef PID_BLOCK_HPP
#define PID_BLOCK_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include <iostream>
#include <cmath>

namespace HEAR{

class PID_Block : public Block{
private:
    InputPort<float>* err_port;
    InputPort<float>* pv_dot_port;
    OutputPort<float>* u_port;
    bool i_term = false, d_term = false, dd_term = false;
    bool en_pv_derivation = false, en_anti_windup = false;
    bool freeze_ = false;
    float prev_err = 0, prev2_err = 0, prev_pv_rate = 0, accum_u = 0, accum_I = 0;
    PID_parameters _parameters;

public:
    enum IP{ERROR};
    enum OP{OUTPUT};
    PID_Block();
    ~PID_Block(){ delete err_port, pv_dot_port, u_port;}
    void process();
    void update(UpdateMsg* u_msg);
    void update_params(PID_parameters* para);
    void reset();
    float pid_direct(float err, float pv_first, float pv_second = 0);
};

}

#endif