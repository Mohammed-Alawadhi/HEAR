#ifndef DIFFERENTIATOR_HPP
#define DIFFERENTIATOR_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

template <class T>
class Differentiator : public Block{
private:
    InputPort<T>* _inp;
    OutputPort<T>* _out;
    T prev_inp, prev_diff;
    int reset_count = 0;
    double _dt;
public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    Differentiator(double dt, int b_id);
    ~Differentiator(){}
    void process();    
};

template <class T>
Differentiator<T>::Differentiator (double dt, int b_uid) : Block(BLOCK_ID::DIFFERENTIATOR, b_uid){
    _inp = createInputPort<T>(0, "INPUT");
    _out = createOutputPort<T>(0, "OUTPUT");
    prev_inp = 0;
    _dt = dt;
    
}

template <class T>
void Differentiator<T>::process(){
    T inp, out;
    inp = 0;
    _inp->read(inp);
    auto dx = inp - prev_inp;
    if(inp == prev_inp){
        inp == 0? out = 0 : out = prev_diff;
        reset_count++;
    }
    else{
        if(reset_count > 0){
            out = dx / (_dt*(reset_count+1));
            reset_count = 0;
        }
        else{
            out = dx/_dt;
        }
        prev_inp = inp;
    }
    prev_diff = out;
    _out->write(out);
}

}
#endif