// TODO: Make it a template

#ifndef MULTIPLY_HPP
#define MULTIPLY_HPP

#include "HEAR_core/Block.hpp"

namespace HEAR {

class Multiply : public Block {
private: 
    InputPort<float>* inp0, *inp1;
    OutputPort<float>* out;

public:
    enum IP{INPUT_0, INPUT_1};
    enum OP{OUTPUT};
    Multiply(int);
    ~Multiply(){}
    void process();
};


}

#endif