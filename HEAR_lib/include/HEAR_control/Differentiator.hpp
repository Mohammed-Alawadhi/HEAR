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
    T prev_inp, prev_diff, _hold, prev_out;
    float _max_diff = 100000;
    double _dt;
    int reset_count = 0;
    bool _sup_peak = false;
public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    Differentiator(double dt, int b_id);
    ~Differentiator(){}
    void process();
    void supPeak(float peak_val){
        _max_diff = abs(peak_val);
        _sup_peak = true;
    }    
};

template <class T>
Differentiator<T>::Differentiator (double dt, int b_uid) : Block(BLOCK_ID::DIFFERENTIATOR, b_uid){
    _inp = createInputPort<T>(0, "INPUT");
    _out = createOutputPort<T>(0, "OUTPUT");
    prev_inp = 0; prev_diff = 0; prev_out = 0;_hold = 0;
    _dt = dt;
}

template <class T>
void Differentiator<T>::process(){
    T inp, out, diff;
    inp = 0;
    _inp->read(inp);
    auto dx = inp - prev_inp;

    if(inp == prev_inp){
        diff = prev_out;
        reset_count++;
    }
    else{
        if(reset_count > 0){
            diff = dx / (_dt*(reset_count+1));
            reset_count = 0;
        }
        else{
            diff= dx/_dt;
        }
        prev_inp = inp;
    }
    out = diff;
    if(_sup_peak){
        if((diff -prev_diff)  > _max_diff){
            out = _hold;
        } 
        else if((diff - prev_diff) < -_max_diff){
            out = _hold;
        }
        else{
            _hold = diff; 
        }
    }
    prev_out = out;
    prev_diff = diff;
    _out->write(out);
}

}
#endif