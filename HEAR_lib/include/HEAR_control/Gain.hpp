#ifndef GAIN_HPP
#define GAIN_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{

class Gain : public Block {
private: 
    float K;
    InputPort<float>* inp;
    OutputPort<float>* out;

public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    Gain(const float&);
    ~Gain(){}
    void process();
};

}

#endif

