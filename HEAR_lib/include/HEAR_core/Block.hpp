
#pragma once

#include <map>
#include <string>

#include "Port.hpp"

// namespace HEAR{

class Block{
public:
    int _block_uid;
    enum IP{};
    enum OP{}; 
    Block(int block_id);
    virtual ~Block();
    template <class T> InputPort<T>* createInputPort(IP idx, TYPE dtype, std::string name);
    template <class T> OutputPort<T>* createOutputPort(OP idx, TYPE dtype, std::string name);
    template <class T> InputPort<T>* getInputPort(Block::IP idx);
    template <class T> OutputPort<T>* getOutputPort(Block::OP idx);
    virtual std::string getInputPortName(Block::IP idx);
    virtual std::string getOutputPortName(Block::OP idx);    
    virtual const std::map<Block::IP, Port*> getInputPorts() const;
    virtual const std::map<Block::OP, Port*> getOutputPorts() const;
    virtual void process() = 0;
    virtual int getBlockID();

private:
    int _block_id;
    std::map<Block::IP, Port*> _input_ports;
    std::map<Block::OP, Port*> _output_ports;
    std::map<Block::IP, std::string> _input_port_names;
    std::map<Block::OP, std::string> _output_port_names;
    
};

Block::Block(int block_id) : _block_id(block_id){}

int Block::getBlockID(){ return _block_id; }

template <class T> 
InputPort<T>* Block::createInputPort(Block::IP idx, TYPE dtype, std::string name){
    auto port = new InputPort<T>(dtype);
    _input_ports.emplace(idx, port);
    _input_port_names.emplace(idx, name);
    return port;
}

template <class T> 
OutputPort<T>* createOutputPort(Block::OP idx, TYPE dtype, std::string name){
    auto port  = new OutputPort<T>(dtype);
    _output_ports.emplace(idx, port);
    _output_port_names.emplace(idx, name);
    return port;
}

template <class T> 
InputPort<T>* getInputPort(Block::IP idx){
    return (InputPort<T>*)_input_ports[idx];
}
template <class T> 
OutputPort<T>* getOutputPort(Block::OP idx){
    return (OutputPort<T>*)_output_ports[idx];
}

std::string Block::getInputPortName(Block::IP idx){
    return _input_port_names[idx];
}

std::string Block::getOutputPortName(Block::OP idx){
    return _output_port_names[idx];
}

// }