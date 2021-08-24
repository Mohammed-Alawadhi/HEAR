#include "HEAR_control/InvertedSwitch3.hpp"

namespace HEAR {

InvertedSwitch3::InvertedSwitch3(int b_uid) : Block(BLOCK_ID::INVERTED_SWITCH3, b_uid){
    _default_port = createInputPort<Vector3D<float>>(IP::NC, "NC");
    _other_port = createInputPort<Vector3D<float>>(IP::NO, "NO");
    _output_port = createOutputPort<Vector3D<float>>(OP::COM, "COM");
}

void InvertedSwitch3::process(){
    Vector3D<float> data;
    if(_triggered){
        _other_port->read(data);
    }
    else {
        _default_port->read(data);
    }
    _output_port->write(data);
}

void InvertedSwitch3::update(UpdateMsg* u_msg){
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