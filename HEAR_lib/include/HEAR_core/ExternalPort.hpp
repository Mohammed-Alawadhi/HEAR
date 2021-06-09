#ifndef EXTERNALPORT_HPP
#define EXTERNALPORT_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Port.hpp"
#include <string>

namespace HEAR{

template <class T>
class ExternalOutputPort: public Block{
private:
    T _data;
    InputPort<T>* _in;
public:
    enum IP{INUPT}; 
    ExternalOutputPort(int b_uid) : Block(BLOCK_ID::EXT_OP, b_uid){
        _in = this->createInputPort<T>(0, "EXT_INPUT");
    }
    std::mutex mtx;
    void read(T &data){
        mtx.lock();
        data = _data;
        mtx.unlock();
    }
    void write(const T &data){
        mtx.lock();
        this->_data = data;
        mtx.unlock();
    }
    void process(){
        T data;
        this->_in->read(data);
        write(data);
    }
};

template <class T>
class ExternalInputPort: public Block{
private:
    ExternalOutputPort<T>* _connected_port = NULL;
    OutputPort<T>* _out;
public:
    enum OP{OUTPUT};
    ExternalInputPort(int b_uid) : Block(BLOCK_ID::EXT_IP, b_uid) {
        _out = createOutputPort<T>(0, "EXT_OUTPUT");
    }
    void read(T &data){
        assert(_connected_port != NULL);
        _connected_port->read(data);
    }
    void process(){
        T temp;
        this->read(temp);
        this->_out->write(temp);
    }
    void connect(ExternalOutputPort<T>* port){
        this->_connected_port = port;
    }
};

}

#endif