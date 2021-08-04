
#include "HEAR_control/Gain.hpp"

namespace HEAR{

Gain::Gain (int b_uid) : Block(BLOCK_ID::GAIN, b_uid) {
    inp = createInputPort<float>(IP::INPUT, "INPUT");
    out = createOutputPort<float>(OP::OUTPUT, "OUTPUT");
    K = 1;
}

void Gain::process(){
    float x = 0;
    inp->read(x);
    out->write(K*x);
}
    
} // namespace HEAR