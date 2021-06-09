#include "HEAR_control/Saturation.hpp"

namespace HEAR{

Saturation::Saturation(int b_uid) : Block(BLOCK_ID::SATURATION, b_uid){
    _inp_port = createInputPort<float>(IP::INPUT, "INPUT");
    _out_port = createOutputPort<float>(OP::OUTPUT, "OUTPUT");
}

void Saturation::process(){
    float data;
    _inp_port->read(data);
    if(data > clip_value){
        data = clip_value;
    }
    else if(data < -clip_value){
        data = -clip_value;
    }
    _out_port->write(data);
}

}