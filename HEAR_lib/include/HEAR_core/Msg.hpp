#include "DataTypes.hpp"

#include <vector>

namespace HEAR{
class Msg{
protected:
    TYPE _type = TYPE::NA;
public:
    Msg(TYPE type) : _type(type) {}
    virtual ~Msg(){}
    virtual TYPE getType() const { return _type;};
};

class FloatMsg : public Msg{
public:
    float data;
    FloatMsg() : Msg(TYPE::Float){}
    ~FloatMsg(){}
    // TYPE getType(){ return this->_type; }
};

class Float3Msg : public Msg{
public:
    std::vector<float> data;
    Float3Msg() : Msg(TYPE::Float3){}
    ~Float3Msg(){}
    // TYPE getType(){ return this->_type;}
};




}