#include "HEAR_control/FromHorizon.hpp"

namespace HEAR{

FromHorizon::FromHorizon(int b_uid) : Block(BLOCK_ID::FROMHORIZON, b_uid){
    yaw_port = createInputPort<float>(IP::YAW, "YAW");
    inp_vec_port = createInputPort<Vector3D<float>>(IP::INP_VEC, "INP_VEC");
    out_vec_port = createOutputPort<Vector3D<float>>(OP::OUT_VEC, "OUT_VEC");
}

void FromHorizon::process(){
    Vector3D<float> data(0, 0, 0);
    float yaw =0;
    yaw_port->read(yaw);
    Rot.setEulerYPR(yaw, 0.0, 0.0);
    inp_vec_port->read(data);
    auto rotated_vec =  Rot.transpose()*tf2::Vector3(data.x, data.y, data.z);
    out_vec_port->write(Vector3D<float>((float)rotated_vec.x(), (float)rotated_vec.y(), (float)rotated_vec.z()));
}

}