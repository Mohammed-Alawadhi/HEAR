#include "HEAR_control/Mux3.hpp"

namespace HEAR{

Mux3::Mux3(int b_uid) : Block(BLOCK_ID::MUX3, b_uid){
    _ip1 = createInputPort<float>(IP::X, "X");
    _ip2 = createInputPort<float>(IP::Y, "Y");
    _ip3 = createInputPort<float>(IP::Z, "Z");
    _op = createOutputPort<Vector3D<float>>(OP::OUTPUT, "OUTPUT");
}

void Mux3::process(){
    Vector3D<float> out;
    _ip1->read(out.x);
    _ip2->read(out.y);
    _ip3->read(out.z);
    _op->write(out);
}

}