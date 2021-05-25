#pragma once

#include "Port.hpp"
#include <vector>
#include <string>

namespace HEAR{

class Block{
private:
    int _block_id;
    std::vector<Port*> _input_ports;
    std::vector<Port*> _output_ports;
    std::vector<std::string> _iport_names;
    std::vector<std::string> _oport_names;

public:
    int _block_uid;
    enum IP{};
    enum OP{}; 
//    std::vector<std::vector<Port*>*> _ports;
    Block(int block_id);
    void createPorts(int num_ips, int num_ops);
    template <class T> InputPort<T>* createInputPort(int idx, int dtype, std::string name);
    template <class T> OutputPort<T>* createOutputPort(int idx, int dtype, std::string name);
    template <class T> InputPort<T>* getInputPort(int idx);
    template <class T> OutputPort<T>* getOutputPort(int idx);
    virtual std::string getInputPortName(int idx);
    virtual std::string getOutputPortName(int idx);    
    virtual ~Block();
    virtual std::vector<Port*> getInputPorts() const;
    virtual std::vector<Port*> getOutputPorts() const;
    virtual void process();
    virtual int getBlockID();
};

}
