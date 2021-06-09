
#ifndef SATURATION_HPP
#define SATURATION_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{

class Saturation : public Block {
private:
    InputPort<float>* _inp_port;
    OutputPort<float>* _out_port;
    float clip_value;
public:
    enum IP{INPUT};
    enum OP{OUTPUT};
    Saturation(int b_uid);
    void setClipValue(const float& clip_val){ clip_value = clip_val;}
    ~Saturation(){}
    void process();
};

}

#endif