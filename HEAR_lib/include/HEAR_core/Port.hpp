#ifndef PORT_HPP
#define PORT_HPP

#include <mutex>
#include <atomic>
#include <vector>
#include <cassert>

#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

class Port{
private:
    TYPE _dtype = TYPE::NA;
protected:
    int _connected_block_uid= -1;
public:
    int _host_block_uid = -1;
    Port (TYPE dtype) : _dtype(dtype){}
    virtual ~Port(){}
    virtual TYPE getType() { return _dtype; }
    virtual int getConnectedBlockUID(){ return _connected_block_uid; }
};

template <class T> class OutputPort : public Port{
public:
    T _data;
    OutputPort(TYPE dtype, int host_block_uid) : Port(dtype) {
        _host_block_uid = host_block_uid;
    }
    void write(T const &data) { this->_data = data; }
};

template <class T> class InputPort : public Port{
private:
    OutputPort<T>* _connected_port = NULL;
public:
    InputPort(TYPE dtype) : Port(dtype){}

    void read(T &data){
        if (_connected_port == NULL){
            //print warning
        }
        else{
            data = _connected_port->_data;
        }
    }

    void connect(OutputPort<T>* port) {
        this->_connected_port = port;
        this->_connected_block_uid = port->_host_block_uid;
    }
};


}

#endif