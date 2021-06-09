#ifndef PORT_HPP
#define PORT_HPP

#include <mutex>
#include <atomic>
#include <vector>
#include <cassert>

#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

class Port{
protected:
    int _connected_block_uid= -1;
    int _port_uid = 0;
public:
    virtual ~Port(){}
    virtual IOTYPE getType() = 0;
    virtual int getConnectedBlockUID(){ return _connected_block_uid; }
    virtual int getHostBlockUID(){return _port_uid/256;}
    virtual int getPortID(){ return _port_uid%256;}
    virtual int getPortUID(){return _port_uid;}
};

template <class T> class OutputPort : public Port{
private:
    T _data;
public:
    OutputPort(int host_block_uid, uint8_t port_idx) {
        _port_uid = host_block_uid*256 + port_idx;
    }

    void write(T const &data) { this->_data = data; }

    void get_data(T& data){data = _data;}
    
    IOTYPE getType(){return IOTYPE::OUTPUT;}
};

template <class T> class InputPort : public Port{
private:
    OutputPort<T>* _connected_port = NULL;
public:
    InputPort(int host_block_uid, uint8_t port_idx) {
        _port_uid = host_block_uid*256 + port_idx;
    }
    void read(T &data){
        if (_connected_port == NULL){
            //print warning
        }
        else{
            _connected_port->get_data(data);
        }
    }

    void connect(OutputPort<T>* port) {
        this->_connected_port = port;
        this->_connected_block_uid = port->getHostBlockUID();
    }

    IOTYPE getType(){return IOTYPE::INPUT;}
};


}

#endif