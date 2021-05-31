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
public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    Differentiator(TYPE dtype);
    ~Differentiator(){}
    void process();    
};

template <class T>
Differentiator<T>::Differentiator (TYPE dtype) : Block(BLOCK_ID::DIFFERENTIATOR){
    _inp = createInputPort<T>(0, dtype, "INPUT");
    _out = createOutputPort<T>(0, dtype, "OUTPUT");
    
}

template <class T>
void Differentiator<T>::process(){
    T inp;
    _inp->read(inp);
    T out  = (inp - prev_inp)/_dt;
    _out->write(out);
}

}
#endif