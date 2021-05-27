
#pragma once

#include "Block.hpp"
#include "DataTypes.hpp"
#include "Port.hpp"
#include "Msg.hpp"

namespace HEAR{

class ExternalPort: public Block{
private:
    TYPE _dtype = TYPE::NA;
public:
    ExternalPort(int eport_id, int num_ip, int num_op, TYPE dtype) : _dtype(dtype), Block(eport_id, num_ip, num_op){}
    enum IP{INPUT};
    enum OP{OUTPUT};
    virtual ~ExternalPort(){}
    virtual TYPE getType() const {return _dtype;}
    virtual void process(){}
};

class ExternalOutputPort: public ExternalPort{
private:
    InputPort* _in;
public:
    Msg* buffer;
    ExternalOutputPort(Msg* container) : buffer(container), ExternalPort(BLOCK_ID::EXT_OP, 1, 0, container->getType()){}
    std::mutex mtx;
    void read(Msg &data);
    void update(const Msg &data);
    void process();
};

class ExternalInputPort: public ExternalPort{
public:
    ExternalInputPort(TYPE dtype) : ExternalPort(BLOCK_ID::EXT_IP, 1,0, dtype){
        _out = createOutputPort(OP::OUTPUT, new FloatMsg, "IP");
    }
    Msg* read();
    void process();
    void connect(ExternalOutputPort* port);
private:
    ExternalOutputPort* _connected_port = NULL;
    OutputPort* _out;
};



}