
#include "HEAR_core/Block.hpp"

namespace HEAR{

void Block::createPorts(int num_ip, int num_op){
    _input_ports.resize(num_ip);
    _output_ports.resize(num_op);
}

InputPort* Block::createInputPort(int idx, TYPE dtype, std::string name){
    _iport_names[idx] = name;
    auto iport = new InputPort(dtype);
    _input_ports[idx] = iport;
    return iport;
}

OutputPort* Block::createOutputPort(int idx, Msg* container, std::string name){
    _oport_names[idx] = name;
    auto oport = new OutputPort(container, getBlockUID());
    _output_ports[idx] = oport;
    return oport;
}

}