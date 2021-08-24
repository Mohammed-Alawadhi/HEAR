#ifndef FOH_HPP
#define FOH_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

template <class T>
class FoH : public Block{
private:
    InputPort<T>* _inp;
    OutputPort<T>* _out;
    T prev_inp, _hold_val, _der;
    float _max_der = 0.5;
    bool is_first = true;
    double _dt;
public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    FoH(double dt, int b_id);
    ~FoH(){}
    void process();    
};

template <class T>
FoH<T>::FoH (double dt, int b_uid) : Block(BLOCK_ID::FOH, b_uid){
    _inp = createInputPort<T>(0, "INPUT");
    _out = createOutputPort<T>(0, "OUTPUT");
    prev_inp = 0; _hold_val = 0; _der = 0;
    _dt = dt;
    
}

template <class T>
void FoH<T>::process(){
    T inp, out;
    inp = 0;
    _inp->read(inp);
    
    out = inp;
    if (_hold_val == inp){
        if(_der > _max_der){
            _der = _max_der;
        } else if (_der < -_max_der){
            _der = -_max_der;
        }
       out = _dt*_der + prev_inp;
    }  
    else{
        _der = (inp - _hold_val)/_dt;
        _hold_val = inp;
    }
    prev_inp = out;
    
    _out->write(out);
}

}
#endif