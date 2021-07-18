#include "HEAR_control/Switch.hpp"

namespace HEAR {

Switch::Switch(int b_uid) : Block(BLOCK_ID::SWITCH, b_uid){
    _input_port = createInputPort<float>(IP::COM, "COM");
    _default_port = createOutputPort<float>(OP::NC, "NC"); _default_port->write(0);
    _other_port = createOutputPort<float>(OP::NO, "NO"); _other_port->write(0);
}

void Switch::process(){
    float data;
    _input_port->read(data);
    if(_triggered){
        _other_port->write(data);
    }
    else {
        _default_port->write(data);
    }
}

void Switch::update(UpdateMsg* u_msg){
    if(u_msg->getType() == UPDATE_MSG_TYPE::SWITCH_TRIG){
        switch (((SwitchMsg*)u_msg)->sw_state)
        {
        case SWITCH_STATE::ON :
            _triggered = true;
            break;
        case SWITCH_STATE::OFF :
            _triggered = false;
            break;
        case SWITCH_STATE::TOGGLE :
            _triggered == true? _triggered = false : _triggered = true;
            break;
        default:
            break;
        }
    }
    if(u_msg->getType() == UPDATE_MSG_TYPE::BOOL_MSG){
        _triggered = ((BoolMsg*)u_msg)->data;
    }
}

}