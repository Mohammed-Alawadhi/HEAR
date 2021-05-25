
#include "HEAR_core/ExternalPort.hpp"

namespace HEAR{

int ExternalPort::getType(){
    return _dtype;
}

template <class T>
ExternalInputPort<T>::ExternalInputPort(int dtype){
    createPorts(0, 1);
    _out = createOutputPort<T>(OP::OUTPUT, dtype, "IP");
    
    this->_dtype = dtype;
}

template <class T>
void ExternalInputPort<T>::connect(ExternalOutputPort<T>* port){
    this->_connected_port = port;
}

template <class T>
void ExternalInputPort<T>::process(){
    if(_connected_port != NULL){
        _connected_port->mtx.lock();
        this->_out->_data = _connected_port->_data;
        _connected_port->mtx.unlock();
    }
    else{
    //    raise exception;
    }

}

template <class T> 
ExternalOutputPort<T>::ExternalOutputPort(int dtype){
    createPorts(1,0);
    _in = createInputPort<T>(IP::INPUT, dtype, "OP");

    this->_dtype = dtype;
}

template <class T>
void ExternalOutputPort<T>::process(){
    mtx.lock();
    this->_in->read(this->_data);
    mtx.unlock();
}

}