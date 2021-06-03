#include "HEAR_control/Sum.hpp"

namespace HEAR{

Sum::Sum (OPERATION op) : _op(op), Block(BLOCK_ID::SUM){
    operand1_port = createInputPort<float>(IP::OPERAND1, TYPE::Float, "OPERAND1");
    operand2_port = createInputPort<float>(IP::OPERAND2, TYPE::Float, "OPERAND2");
    out = createOutputPort<float>(OP::OUTPUT, TYPE::Float, "OUTPUT");
}

void Sum::process(){
    float operand1, operand2;
    operand1_port->read(operand1);
    operand2_port->read(operand2);
    if (_op == OPERATION::ADD){
        out->write(operand1+operand2);
    }
    else if (_op == OPERATION::SUB){
        out->write(operand1 - operand2);
    }
}

}