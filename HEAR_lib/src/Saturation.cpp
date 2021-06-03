#include "HEAR_control/Saturation.hpp"

namespace HEAR{

Saturation::Saturation(const float &clip_val) : clip_value(clip_val), Block(BLOCK_ID::SATURATION){
    _inp_port = createInputPort<float>(IP::INPUT, TYPE::Float, "INPUT");
    _out_port = createOutputPort<float>(OP::OUTPUT, TYPE::Float, "OUTPUT");
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