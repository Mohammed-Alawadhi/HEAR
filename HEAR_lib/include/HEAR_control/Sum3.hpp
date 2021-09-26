
#ifndef SUM3_HPP
#define SUM3_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"


namespace HEAR{

class Sum3 : public Block {
public:
    enum OPERATION {ADD, SUB};
    enum IP{OPERAND1, OPERAND2};
    enum OP{OUTPUT};
    Sum3(int b_uid);
    void setOperation(OPERATION op) { _op = op;}
    ~Sum3(){delete operand1_port, operand2_port, out;}
    void process();
private:
    OPERATION _op;
    InputPort<Vector3D<float>> *operand1_port, *operand2_port;
    OutputPort<Vector3D<float>>* out;

};

}

#endif