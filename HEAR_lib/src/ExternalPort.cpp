
#include "HEAR_core/ExternalPort.hpp"

namespace HEAR{


void ExternalInputPort::connect(ExternalOutputPort* port){
    assert(port->getType() == this->getType());
    this->_connected_port = port;
}

void ExternalInputPort<T>::read(T &data){
    if(_connected_port != NULL){
        T data;
        _connected_port->read(data);      
    }
    else{
    //    raise exception;
    }

}

template <class T>
void ExternalInputPort<T>::process(){
    T data;
    this->read(data);
    this->_out->write(data);

}

template <class T> 
ExternalOutputPort<T>::ExternalOutputPort(int dtype) : ExternalPort(BLOCK_ID::EXT_OP){
    createPorts(1,0);
    _in = createInputPort<T>(IP::INPUT, dtype, "OP");

    this->_dtype = dtype;
}

template <class T>
void ExternalOutputPort<T>::read(T &data){
    mtx.lock();
    data = _data;
    mtx.unlock();
}

void ExternalOutputPort::update(const Msg &data){
    mtx.lock();
    this->_data = data;
    mtx.unlock();
}

template <class T>
void ExternalOutputPort<T>::process(){
    T data;
    this->_in->read(data);
    update(data);
}
}