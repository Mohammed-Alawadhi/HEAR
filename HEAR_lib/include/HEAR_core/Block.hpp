
#ifndef BLOCK_HPP
#define BLOCK_HPP

#include <map>
#include <string>

#include "HEAR_core/Port.hpp"

namespace HEAR{

class Block{
public:
    int _block_uid;
    Block(BLOCK_ID block_id) : _block_id(block_id){}
    virtual ~Block(){}
    template <class T> InputPort<T>* createInputPort(int idx, TYPE dtype, std::string name);
    template <class T> OutputPort<T>* createOutputPort(int idx, TYPE dtype, std::string name);
    template <class T> InputPort<T>* getInputPort(int idx){ return (InputPort<T>*)_input_ports[idx]; }
    template <class T> OutputPort<T>* getOutputPort(int idx) { return (OutputPort<T>*)_output_ports[idx];}
    virtual std::string getInputPortName(int idx) { return _input_port_names[idx];}
    virtual std::string getOutputPortName(int idx) {return _output_port_names[idx];}    
    virtual const std::map<int, Port*> getInputPorts() const {return _input_ports;}
    virtual const std::map<int, Port*> getOutputPorts() const{return _output_ports;}
    virtual void process() = 0;
    void set_dt(float dt) { _dt = dt;}
    virtual int getBlockID(){ return _block_id;}
    virtual void reset(){}
    virtual void update(UpdateMsg* u_msg){}

protected:
    float _dt;
private:
    BLOCK_ID _block_id;
    std::map<int, Port*> _input_ports;
    std::map<int, Port*> _output_ports;
    std::map<int, std::string> _input_port_names;
    std::map<int, std::string> _output_port_names;
    
};

template <class T> 
InputPort<T>* Block::createInputPort(int idx, TYPE dtype, std::string name){
    auto port = new InputPort<T>(dtype);
    _input_ports.emplace(idx, port);
    _input_port_names.emplace(idx, name);
    return port;
}

template <class T> 
OutputPort<T>* Block::createOutputPort(int idx, TYPE dtype, std::string name){
    auto port  = new OutputPort<T>(dtype, this->_block_uid);
    _output_ports.emplace(idx, port);
    _output_port_names.emplace(idx, name);
    return port;
}


}

#endif