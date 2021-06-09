#ifndef DEMUX3_HPP
#define DEMUX3_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Vector3D.hpp"

namespace HEAR{

class Demux3 : public Block {

private:
    InputPort<Vector3D<float>>* _inp;
    OutputPort<float> *_op1, *_op2, *_op3;

public:
    enum IP{INPUT};
    enum OP{X, Y, Z};
    Demux3(int b_uid);
    ~Demux3(){}
    void process();

};

}

#endif