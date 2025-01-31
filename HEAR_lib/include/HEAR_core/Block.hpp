
#ifndef BLOCK_HPP
#define BLOCK_HPP

#include <map>
#include <string>
#include <iostream>

#include "HEAR_core/Port.hpp"

namespace HEAR{

class Block{
public:
    Block(BLOCK_ID block_id, int b_uid) : _block_id(block_id), _block_uid(b_uid)  {}
    virtual ~Block(){
        // for (const auto& port : _input_ports ){
        //     delete port.second;    
        // }
        // for (const auto& port : _output_ports ){
        //     delete port.second;    
        // }
    }
    template <class T> InputPort<T>* createInputPort(int idx, std::string name);
    template <class T> OutputPort<T>* createOutputPort(int idx, std::string name);
    template <class T> InputPort<T>* getInputPort(int idx){ return (InputPort<T>*)_input_ports[idx]; }
    template <class T> OutputPort<T>* getOutputPort(int idx) { return (OutputPort<T>*)_output_ports[idx];}
    virtual std::string getInputPortName(int idx) { return _input_port_names[idx];}
    virtual std::string getOutputPortName(int idx) {return _output_port_names[idx];}    
    virtual const std::map<int, Port*> getInputPorts() const {return _input_ports;}
    virtual const std::map<int, Port*> getOutputPorts() const{return _output_ports;}
    virtual void process() = 0;
    int getBlockUID() const {return _block_uid;}
    int getBlockID() const { return _block_id;}
    virtual void reset(){}
    virtual void update(UpdateMsg* u_msg){}

private:
    int _block_uid;
    BLOCK_ID _block_id;
    std::map<int, Port*> _input_ports;
    std::map<int, Port*> _output_ports;
    std::map<int, std::string> _input_port_names;
    std::map<int, std::string> _output_port_names;
    
};

template <class T> 
InputPort<T>* Block::createInputPort(int idx, std::string name){
    auto port = new InputPort<T>(this->_block_uid, idx);
    _input_ports.emplace(idx, port);
    _input_port_names.emplace(idx, name);
    return port;
}

template <class T> 
OutputPort<T>* Block::createOutputPort(int idx, std::string name){
    auto port  = new OutputPort<T>(this->_block_uid, idx);
    _output_ports.emplace(idx, port);
    _output_port_names.emplace(idx, name);
    return port;
}


}

#endif