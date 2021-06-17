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
    T prev_inp;
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
    T inp;
    inp = 0;
    _inp->read(inp);
    T out  = (inp - prev_inp)/_dt;
    prev_inp = inp;
    _out->write(out);
}

}
#endif