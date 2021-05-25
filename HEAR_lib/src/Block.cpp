
#include "HEAR_core/Block.hpp"

namespace HEAR{

Block::Block(int block_id){
    _block_id = block_id;
}
Block::~Block(){
//    _ports = {&_input_ports, &_output_ports};
}

void Block::process(){}

void Block::createPorts(int num_ips, int num_ops){
    _input_ports = std::vector<Port*>(num_ips);
    _output_ports = std::vector<Port*>(num_ops);
    _iport_names = std::vector<std::string>(num_ips);
    _oport_names = std::vector<std::string>(num_ops);
}

template <class T>
InputPort<T>* Block::createInputPort(int idx, int dtype, std::string name){
    _iport_names[idx] = name;
    _input_ports[idx] = new InputPort<T>(Port::getPortUID(this->_block_uid, idx, IOTYPE::INPUT), dtype);
    return _input_ports[idx];
}

template <class T>
OutputPort<T>* Block::createOutputPort(int idx, int dtype, std::string name){
    _oport_names[idx] = name;
    _output_ports[idx] = new OutputPort<T>(Port::getPortUID(this->_block_uid, idx, IOTYPE::OUTPUT), dtype, this->_block_uid);
    return _output_ports[idx];
}

template <class T> 
InputPort<T>* Block::getInputPort(int idx){
    return _input_ports[idx];
}

template <class T> 
OutputPort<T>* Block::getOutputPort(int idx){
    return _output_ports[idx];
}

std::vector<Port*> Block::getInputPorts() const{
    return _input_ports;
}

std::vector<Port*> Block::getOutputPorts() const{
    return _output_ports;
}

std::string Block::getInputPortName(int idx){
    return _iport_names[idx];
}

std::string Block::getOutputPortName(int idx){
    return _oport_names[idx];
}

int Block::getBlockID(){
    return _block_id;
}

}