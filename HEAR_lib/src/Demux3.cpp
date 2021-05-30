#include "HEAR_control/Demux3.hpp"

namespace HEAR{

Demux3::Demux3() : Block(BLOCK_ID::DEMUX3) {
    _inp = createInputPort<float3>(IP::INPUT, TYPE::Float3, "INPUT");
    _op1 = createOutputPort<float>(OP::X, TYPE::Float, "X");
    _op2 = createOutputPort<float>(OP::Y, TYPE::Float, "Y");
    _op3 = createOutputPort<float>(OP::Z, TYPE::Float, "Z");

}

void Demux3::process(){
    float3 inp;
    _inp->read(inp);
    _op1->write(inp.x);
    _op2->write(inp.y);
    _op3->write(inp.z);
}

}