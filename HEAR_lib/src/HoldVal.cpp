#include "HEAR_control/HoldVal.hpp"

namespace HEAR {

HoldVal::HoldVal(int b_uid) : Block(BLOCK_ID::HOLDVAL, b_uid) {
    inp = createInputPort<float>(IP::INPUT, "INPUT");
    out = createOutputPort<float>(OP::OUTPUT, "OUTPUT");
}

void HoldVal::process(){
    if(!_hold){
        inp->read(_val);
        out->write(_val);
    }
}

void HoldVal::update(UpdateMsg* u_msg){
    if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
        inp->read(_val);
        _hold = ((BoolMsg*)u_msg)->data;
    }
}

}