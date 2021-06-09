#include "HEAR_control/Demux3.hpp"

namespace HEAR{

Demux3::Demux3(int b_uid) : Block(BLOCK_ID::DEMUX3, b_uid) {
    _inp = createInputPort<Vector3D<float>>(IP::INPUT, "INPUT");
    _op1 = createOutputPort<float>(OP::X, "X");
    _op2 = createOutputPort<float>(OP::Y, "Y");
    _op3 = createOutputPort<float>(OP::Z, "Z");

}

void Demux3::process(){
    Vector3D<float> inp(0, 0, 0);
    _inp->read(inp);
    _op1->write(inp.x);
    _op2->write(inp.y);
    _op3->write(inp.z);
}

}