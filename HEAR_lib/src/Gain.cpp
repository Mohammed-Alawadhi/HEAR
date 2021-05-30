
#include "HEAR_control/Gain.hpp"

namespace HEAR{

Gain::Gain (const float& gain) : K(gain), Block(BLOCK_ID::GAIN) {
    inp = createInputPort<float>(IP::INPUT, TYPE::Float, "INPUT");
    out = createOutputPort<float>(OP::OUTPUT, TYPE::Float, "OUTPUT");
}

void Gain::process(){
    float x;
    inp->read(x);
    out->write(K*x);
}
    
} // namespace HEAR