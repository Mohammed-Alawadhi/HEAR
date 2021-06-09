
#ifndef SUM_HPP
#define SUM_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
namespace HEAR{

class Sum : public Block {
public:
    enum OPERATION {ADD, SUB};
    enum IP{OPERAND1, OPERAND2};
    enum OP{OUTPUT};
    Sum(int b_uid);
    void setOperation(OPERATION op) { _op = op;}
    ~Sum(){delete operand1_port, operand2_port, out;}
    void process();
private:
    OPERATION _op;
    InputPort<float> *operand1_port, *operand2_port;
    OutputPort<float>* out;

};

}

#endif