#ifndef Constant_HPP
#define Constant_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

template <class T>
class Constant : public Block{
private:
    OutputPort<T>* _out;
    T _val;
public:
    enum OP{OUTPUT};
    Constant(int);
    ~Constant(){}
    void setValue(const T& val);
    void process();    
};

template <class T>
Constant<T>::Constant (int b_uid) : Block(BLOCK_ID::CONSTANT, b_uid){
    _out = createOutputPort<T>(0, "OUTPUT");
    _val = 0;    
}

template<class T>
void Constant<T>::setValue(const T& val){
    _val = val;
}

template <class T>
void Constant<T>::process(){
    _out->write(_val);
}

}
#endif