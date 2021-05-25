
#pragma once

#include "Block.hpp"
#include "DataTypes.hpp"
#include "Port.hpp"

namespace HEAR{

class ExternalPort: public Block{

public:
    ExternalPort(int eport_id);
    enum IP{INPUT};
    enum OP{OUTPUT};
    virtual ~ExternalPort(){}
    int _dtype = TYPE::NA;
    int getType();
    virtual void process(){}
};

template <class T>
class ExternalOutputPort: public ExternalPort{
private:
    T _data;
    InputPort<T>* _in;
public:
    ExternalOutputPort(int dtype);
    std::mutex mtx;
    void read(T &data);
    void update(const T &data);
    void process();
};

template <class T>
class ExternalInputPort: public ExternalPort{
public:
    ExternalInputPort(int dtype);
    void read(T &data);
    void process();
    void connect(ExternalOutputPort<T>* port);
private:
    ExternalOutputPort<T>* _connected_port = NULL;
    OutputPort<T>* _out;
};
}