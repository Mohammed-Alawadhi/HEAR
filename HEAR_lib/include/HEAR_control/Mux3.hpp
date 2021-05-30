#ifndef MUX3_HPP
#define MUX3_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"

namespace HEAR{

class Mux3 : public Block {
private:
    InputPort<float> *_ip1, *_ip2, *_ip3;
    OutputPort<float3>* _op;

public:
    enum IP{X, Y, Z};
    enum OP{OUTPUT};
    Mux3();
    ~Mux3(){}
    void process();

};

}
#endif