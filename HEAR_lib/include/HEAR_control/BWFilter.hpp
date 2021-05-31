#ifndef BWFILTER_HPP
#define BWFILTER_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

template <class T>
class BWFilter2 : public Block{
private:
    InputPort<T>* _inp_port;
    OutputPort<T>*  _out_port;
    T prev_y=0, prev2_y=0, prev_x=0, prev2_x=0;
    float coeff_[5];
public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    BWFilter2(const float* coeff, TYPE dtype);
    ~BWFilter2(){}
    void process();

};

template <class T>
BWFilter2<T>::BWFilter2(const float* coeff, TYPE dtype) : Block(BLOCK_ID::BW_FILT2){
    for(int i=0; i<5; i++){
        coeff_[i] = coeff[i];
    }
    _inp_port = createInputPort(0, dtype, "INPUT");
    _out_port = createOutputPort(0, dtype, "OUTPUT");
}

template <class T>
void BWFilter2<T>::process(){
    T x;
    _inp_port->read(x);
	T y = -coeff_[0] * prev_y - coeff_[1] * prev2_y + coeff_[2] * x + coeff_[3] * prev_x + coeff_[4] * prev2_x;
    prev2_y = prev_y;
	prev_y = y;
	prev2_x = prev_x;
	prev_x = x;
    _out_port->write(y);
}

}

#endif