#ifndef DEMUX3_HPP
#define DEMUX3_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{

class Demux3 : public Block {

private:
    InputPort<float3>* _inp;
    OutputPort<float> *_op1, *_op2, *_op3;

public:
    enum IP{INPUT};
    enum OP{X, Y, Z};
    Demux3();
    ~Demux3(){}
    void process();

};

}

#endif