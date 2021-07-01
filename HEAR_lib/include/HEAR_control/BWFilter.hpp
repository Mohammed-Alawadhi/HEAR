#ifndef BWFILTER_HPP
#define BWFILTER_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{

struct BWFilt2_coeff{
    constexpr static float coeff_120Hz_2nd_butter_5hz[5] =  { -1.279632424997809,0.477592250072517,0.049489956268677,0.098979912537354,0.049489956268677 };
    constexpr static float coeff_200Hz_2nd_butter[5] = { -1.97779f, 0.97803f, 6.1006e-05f, 1.2201e-04f, 6.1006e-05f };
    constexpr static float coeff_200Hz_2nd_butter_50hz[5] = {-1.561018075800718, 0.641351538057563, 0.020083365564211, 0.040166731128423, 0.020083365564211};
    constexpr static float coeff_N200C90[5] = {1.5610, 0.6414, 0.8006, 1.6012, 0.8006};
    constexpr static float coeff_N200C60[5] = {0.3695, 0.1958, 0.3913,    0.7827,    0.3913};
    constexpr static float coeff_N200C50[5] = {-1.8e-16,    0.1716,    0.2929,    0.5858,    0.2929};
};

template <class T>
class BWFilter2 : public Block{
private:
    InputPort<T>* _inp_port;
    OutputPort<T>*  _out_port;
    T prev_y, prev2_y, prev_x, prev2_x;
    float coeff_[5];
    bool _enable = true;
public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    BWFilter2(int b_uid) : Block(BLOCK_ID::BW_FILT2, b_uid){
        _inp_port = createInputPort<T>(0, "INPUT");
        _out_port = createOutputPort<T>(0, "OUTPUT");
        setCoeff(BWFilt2_coeff::coeff_N200C50);
        prev_y = 0; prev_x =0; prev2_y = 0; prev2_x = 0;
    }
    void setCoeff(const float* coeff){
        for(int i=0; i<5; i++){
            coeff_[i] = coeff[i];
        }
    }
    ~BWFilter2(){}
    void process(){
        T x;
        _inp_port->read(x);
        if(_enable){
            T y = -coeff_[0] * prev_y - coeff_[1] * prev2_y + coeff_[2] * x + coeff_[3] * prev_x + coeff_[4] * prev2_x;
            prev2_y = prev_y;
            prev_y = y;
            prev2_x = prev_x;
            prev_x = x;
            _out_port->write(y);
        }
        else{
            _out_port->write(x);
        }
    }
    void update(UpdateMsg* u_msg) override{
        if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
            _enable = ((BoolMsg*)u_msg)->data;
            std::cout << "filter " << (_enable? "enabled" : "disabled") << std::endl;
        }
    }

};

}

#endif