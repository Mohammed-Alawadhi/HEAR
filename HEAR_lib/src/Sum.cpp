#include "HEAR_control/Sum.hpp"

namespace HEAR{

Sum::Sum (int b_uid) : Block(BLOCK_ID::SUM, b_uid){
    operand1_port = createInputPort<float>(IP::OPERAND1, "OPERAND1");
    operand2_port = createInputPort<float>(IP::OPERAND2, "OPERAND2");
    out = createOutputPort<float>(OP::OUTPUT, "OUTPUT");
    _op = OPERATION::SUB;
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