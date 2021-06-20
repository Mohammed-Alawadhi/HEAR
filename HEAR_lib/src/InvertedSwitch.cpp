#include "HEAR_control/InvertedSwitch.hpp"

namespace HEAR {

InvertedSwitch::InvertedSwitch(int b_uid) : Block(BLOCK_ID::INVERTED_SWITCH, b_uid){
    _default_port = createInputPort<float>(IP::NC, "NC");
    _other_port = createInputPort<float>(IP::NO, "NO");
    _output_port = createOutputPort<float>(OP::COM, "COM");
}

void InvertedSwitch::process(){
    float data;
    if(_triggered){
        _other_port->read(data);
    }
    else {
        _default_port->read(data);
    }
    _output_port->write(data);
}

void InvertedSwitch::update(UpdateMsg* u_msg){
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

}

}