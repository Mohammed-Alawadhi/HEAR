#pragma once

#include <mutex>
#include <atomic>
#include <vector>
#include "assert.h"

#include "HEAR_core/DataTypes.hpp"
#include "Msg.hpp"

namespace HEAR{

class Port{
protected:
    TYPE _dtype;
    // int _connected_block_uid= -1; 
public:
    // static size_t getPortUID(const int &block_idx, const int &port_idx, int type);
    // static std::vector<int> translatePortUID(const size_t &p_uid);
    // size_t _port_uid;
    Port(TYPE dtype_) : _dtype(dtype_){}
    virtual ~Port(){}
    virtual TYPE getType() const { return _dtype; };
    // virtual int getConnectedBlockUID(){}
};

class OutputPort : public Port{
public:
    Msg* _data;
    int _host_block_uid;
    OutputPort(Msg* container, int host_block_uid) : _host_block_uid(host_block_uid), _data(container), Port(container->getType()) { };
    // void write(Msg* data){ 
    //     assert(_data->getType() == data->getType());
    //     _data = data;
    // }
};

class InputPort : public Port{
private:
    OutputPort* _connected_port = NULL;
public:
    InputPort(TYPE dtype) : Port(dtype){}
    Msg* read() { 
        assert(_connected_port != NULL);
        return _connected_port->_data;
    }
    void connect(OutputPort* port){
        assert(port->getType() == this->getType());
        _connected_port = port;
    }
    int getConnectedBlockUID(){
        assert(_connected_port != NULL);
        return _connected_port->_host_block_uid;
    }    
};



//function definitions

// Port::Port(){}
// Port::~Port(){}
// int Port::getType(){
//     return _dtype;
// }

// size_t Port::getPortUID(const int &block_idx, const int &port_idx, int type){
//     return block_idx*100 + 50*type +port_idx;
// }

// std::vector<int> Port::translatePortUID(const size_t &p_uid){
//     int temp = p_uid % 100;
//     int type, p_idx;
//     if (temp >=50){
//         type = IOTYPE::OUTPUT;
//         p_idx = temp-50;
//     }
//     else{
//         type = IOTYPE::INPUT;
//         p_idx = temp;
//     }
//     std::vector<int> x;
//     x = {(int)(p_uid/100), p_idx, type};
//     return x;
// }

// template <class T>
// OutputPort<T>::OutputPort(size_t uid, int dtype, int host_block_uid){
//     this->_port_uid = uid;
//     this->_dtype = dtype;
//     this->_host_block_uid = host_block_uid;
// }

// template <class T>
// void OutputPort<T>::write(const T &data){
//     this->_data = data;
// }

// template <class T>
// void OutputPort<T>::read(T &data){
//     data = this->_data;
// }


// template <class T>
// InputPort<T>::InputPort(size_t uid, int dtype){
//     this->_port_uid = uid;
//     this->_dtype = dtype;
// }

// template <class T>
// void InputPort<T>::read(T &data){
//     if(_connected_port != NULL){
//         _connected_port->read(data);
//     }
//     else{
//     //    raise exception;
//     }
// }

// template <class T>
// void InputPort<T>::connect(OutputPort<T>* port){
//     this->_connected_port = port;
//     this->_connected_block_uid = port->_host_block_uid;
// }

// template <class T>
// int InputPort<T>::getConnectedBlockUID(){
//     if(_connected_block_uid != -1){
//         return _connected_block_uid; 
//     }
//     else{
//         //raise exception
//     }
//     return -1;
// }

}