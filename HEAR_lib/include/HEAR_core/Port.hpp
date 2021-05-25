#pragma once

#include <mutex>
#include <atomic>
#include <vector>

//#include "HEAR_core/Block.hpp"
#include "HEAR_core/DataTypes.hpp"

namespace HEAR{

class Port{
protected:
    int _connected_block_uid= -1; 
public:
    static size_t getPortUID(const int &block_idx, const int &port_idx, int type);
    static std::vector<int> translatePortUID(const size_t &p_uid);
    size_t _port_uid;
    virtual ~Port(){}
    int _dtype = TYPE::NA;
    int getType();
    virtual int getConnectedBlockUID(){}
};

template <class T> class OutputPort : public Port{
private:
    T _data;
public:
    int _host_block_uid;
    OutputPort(size_t uid, int dtype, int host_block_uid);
    void write(T const &data);
};

template <class T> class InputPort : public Port{
private:
    OutputPort<T>* _connected_port = NULL;
public:
    InputPort(size_t uid, int dtype);
    void read(T &data);
    void connect(OutputPort<T>* port);
    int getConnectedBlockUID();    
};


}