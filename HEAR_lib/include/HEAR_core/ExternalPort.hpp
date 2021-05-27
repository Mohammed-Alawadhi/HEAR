
#include "Block.hpp"
#include "DataTypes.hpp"
#include "Port.hpp"

// namespace HEAR{

// class ExternalPort: public Block{

// public:
//     ExternalPort(int eport_id);
//     enum IP{INPUT};
//     enum OP{OUTPUT};
//     virtual ~ExternalPort(){}
//     int _dtype = TYPE::NA;
//     int getType();
//     virtual void process(){}
// };

template <class T>
class ExternalOutputPort: public Block{
private:
    T _data;
    InputPort<T>* _in;
public:
    TYPE _dtype;
    enum IP{INUPT}; 
    ExternalOutputPort(TYPE dtype) : _dtype(dtype){
        _in = createInputPort(IP::INPUT, dtype, "EXT_INPUT");
    }
    std::mutex mtx;
    void read(T &data){
        mtx.lock();
        data = _data;
        mtx.unlock();
    }
    void update(const T &data){
        mtx.lock();
        this->_data = data;
        mtx.unlock();
    }
    void process(){
        T data;
        this->_in->read(data);
        update(data);
    }
};

template <class T>
class ExternalInputPort: public Block{
private:
    ExternalOutputPort<T>* _connected_port = NULL;
    OutputPort<T>* _out;
public:
    TYPE _dtype;
    enum OP{OUTPUT};
    ExternalInputPort(TYPE dtype) : _dtype(dtype) {
        _out = createOutputPort(OP::OUTPUT, dtype, "EXT_OUTPUT");
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

// }