#pragma once

#include <mutex>
#include <atomic>
#include <vector>

//#include "HEAR_core/Block.hpp"
#include "HEAR_core/DataTypes.hpp"

// namespace HEAR{

class Port{
private:
    TYPE _dtype = TYPE::NA;
protected:
    int _connected_block_uid= -1; 
public:
    Port (TYPE dtype) : _dtype(dtype){}
    virtual ~Port(){}
    virtual TYPE getType() { return _dtype; }
    virtual int getConnectedBlockUID(){ return _connected_block_uid; }
};

template <class T> class OutputPort : public Port{
public:
    T _data;
    int _host_block_uid;
    OutputPort(TYPE dtype, int host_block_uid) : _host_block_uid(host_block_uid) , Port(dtype) {}
    void write(T const &data) { this->_data = data; }
};

template <class T> class InputPort : public Port{
private:
    OutputPort<T>* _connected_port = NULL;
public:
    InputPort(TYPE dtype) : Port(dtype){}

    void read(T &data){
        assert(_connected_port != NULL);
        data = _connected_port->_data;
    }

    void connect(OutputPort<T>* port) {
        this->_connected_port = port;
        this->_connected_block_uid = port->_host_block_uid;
    }
};


// }