
#ifndef SUM_HPP
#define SUM_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/Port.hpp"
namespace HEAR{

class Sum : public Block {
private:
    int _op;
    InputPort<float> *operand1_port, *operand2_port;
    OutputPort<float>* out;
public:
    enum OPERATION {ADD, SUB};
    enum IP{OPERAND1, OPERAND2};
    enum OP{OUTPUT};
    Sum(OPERATION op);
    ~Sum(){delete operand1_port, operand2_port, out;}
    void process();
};

}

#endif