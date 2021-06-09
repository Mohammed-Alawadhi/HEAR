#ifndef MUX3_HPP
#define MUX3_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
#include "HEAR_core/Vector3D.hpp"

namespace HEAR{

class Mux3 : public Block {
private:
    InputPort<float> *_ip1, *_ip2, *_ip3;
    OutputPort<Vector3D<float>>* _op;

public:
    enum IP{X, Y, Z};
    enum OP{OUTPUT};
    Mux3(int b_uid);
    ~Mux3(){}
    void process();

};

}
#endif