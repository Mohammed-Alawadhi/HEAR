#include "HEAR_control/Mux3.hpp"

namespace HEAR{

Mux3::Mux3() : Block(BLOCK_ID::MUX3){
    _ip1 = createInputPort<float>(IP::X, TYPE::Float, "X");
    _ip2 = createInputPort<float>(IP::Y, TYPE::Float, "Y");
    _ip3 = createInputPort<float>(IP::Z, TYPE::Float, "Z");
    _op = createOutputPort<float3>(OP::OUTPUT, TYPE::Float3, "OUTPUT");
}

void Mux3::process(){
    float3 out;
    _ip1->read(out.x);
    _ip2->read(out.y);
    _ip3->read(out.z);
    _op->write(out);
}

}