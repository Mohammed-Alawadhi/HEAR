#include "HEAR_control/Multiply.hpp"

namespace HEAR{

Multiply::Multiply (int b_uid) : Block(BLOCK_ID::MULTIPLY, b_uid) {
    inp0 = createInputPort<float>(IP::INPUT_0, "INPUT");
    inp1 = createInputPort<float>(IP::INPUT_1, "INPUT");
    out = createOutputPort<float>(OP::OUTPUT, "OUTPUT");
}

void Multiply::process(){
    float x = 0, y = 0;
    inp0->read(x);
    inp1->read(y);
    out->write(x*y);
}
    
} // namespace HEAR