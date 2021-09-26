#include "HEAR_control/Sum3.hpp"

namespace HEAR{

Sum3::Sum3 (int b_uid) : Block(BLOCK_ID::SUM3, b_uid){
    operand1_port = createInputPort<Vector3D<float>>(IP::OPERAND1, "OPERAND1");
    operand2_port = createInputPort<Vector3D<float>>(IP::OPERAND2, "OPERAND2");
    out = createOutputPort<Vector3D<float>>(OP::OUTPUT, "OUTPUT");
    _op = OPERATION::SUB;
}

void Sum3::process(){
    Vector3D<float> operand1, operand2;
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