#pragma once

#include "Port.hpp"
#include <vector>
#include <string>

namespace HEAR{

class Block{
private:
    int _block_id;
    int _block_uid;
    std::vector<InputPort*> _input_ports;
    std::vector<OutputPort*> _output_ports;
    std::vector<std::string> _iport_names;
    std::vector<std::string> _oport_names;
    void createPorts(int num_ip, int num_op);

public:

    Block(int block_id, int num_in_ports, int num_out_ports) : _block_id (block_id){ createPorts(num_in_ports, num_in_ports);}
    virtual ~Block(){}
    
    InputPort* createInputPort(int idx, TYPE dtype, std::string name);
    OutputPort* createOutputPort(int idx, Msg* container, std::string name);

    InputPort* getInputPort(int idx) const { return _input_ports[idx]; }
    OutputPort* getOutputPort(int idx) const { return _output_ports[idx]; }
    virtual std::string getInputPortName(int idx) const {return _iport_names[idx];}
    virtual std::string getOutputPortName(int idx) const {return _oport_names[idx];}  
    virtual std::vector<InputPort*> getInputPorts() const {return _input_ports;}
    virtual std::vector<OutputPort*> getOutputPorts() const { return _output_ports;}
    virtual void process() = 0;
    virtual int getBlockID() const { return _block_id; }
    virtual int getBlockUID() const { return _block_uid; }
};


//function definitions


}
