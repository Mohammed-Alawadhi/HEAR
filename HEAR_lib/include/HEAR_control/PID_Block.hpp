#ifndef PID_BLOCK_HPP
#define PID_BLOCK_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include <iostream>
#include <cmath>

namespace HEAR{

class PID_Block : public Block{
private:
    PID_ID _id;
    double _dt;
    InputPort<float>* err_port;
    InputPort<float>* pv_dot_port;
    OutputPort<float>* u_port;
    bool i_term = false, d_term = false, dd_term = false;
    bool en_pv_derivation = false, en_anti_windup = false;
    bool freeze_ = false;
    float prev_err = 0, prev2_err = 0, prev_pv_rate = 0, accum_u = 0, accum_I = 0;
    PID_parameters _parameters;
    void update_params(PID_parameters* para);
    float pid_direct(float err, float pv_first, float pv_second = 0);
public:
    enum IP{ERROR, PV_DOT};
    enum OP{COMMAND};
    PID_Block(double dt, int b_uid);
    void setPID_ID(PID_ID id){ _id = id;}
    ~PID_Block(){}
    void process();
    void update(UpdateMsg* u_msg) override;
    void reset() override;
};

}

#endif