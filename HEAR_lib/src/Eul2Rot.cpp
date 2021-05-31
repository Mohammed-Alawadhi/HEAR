#include "HEAR_control/Eul2Rot.hpp"

namespace HEAR {

Eul2Rot::Eul2Rot() : Block(BLOCK_ID::EUL2ROT){
    _inp_port = createInputPort<Vector3D<float>>(IP::EUL_ANGLES, TYPE::Float3, "EUL_ANGLES");
    _out_port = createOutputPort<tf2::Matrix3x3>(OP::ROT_MAT, TYPE::RotMat, "ROT_MAT");

}

void Eul2Rot::process(){
    Vector3D<float> eul;
    _inp_port->read(eul);
    Rot.setEulerYPR(eul.z, eul.y, eul.x);
    _out_port->write(Rot);
}


}