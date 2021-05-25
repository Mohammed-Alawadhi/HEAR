
#pragma once

#include "Block.hpp"
#include "DataTypes.hpp"
#include "Port.hpp"

namespace HEAR{

class ExternalPort: public Block{

public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    virtual ~ExternalPort(){}
    int _dtype = TYPE::NA;
    int getType();
    virtual void process(){}
};

template <class T>
class ExternalOutputPort: public ExternalPort{
public:
    
    std::mutex mtx;
    T _data;
    InputPort<T>* _in;
    ExternalOutputPort(int dtype);
    void process();
};

template <class T>
class ExternalInputPort: public ExternalPort{
public:
    ExternalInputPort(int dtype);
    OutputPort<T>* _out;
    void process();
    void connect(ExternalOutputPort<T>* port);
private:
    ExternalOutputPort<T>* _connected_port = NULL;

};
}