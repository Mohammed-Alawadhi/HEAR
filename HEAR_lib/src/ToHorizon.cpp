#include "HEAR_control/ToHorizon.hpp"

namespace HEAR{

ToHorizon::ToHorizon() : Block(BLOCK_ID::TOHORIZON){
    yaw_port = createInputPort<float>(IP::YAW, TYPE::Float, "YAW");
    inp_vec_port = createInputPort<Vector3D<float>>(IP::INP_VEC, TYPE::Float3, "INP_VEC");
    out_vec_port = createOutputPort<Vector3D<float>>(OP::OUT_VEC, TYPE::Float3, "OUT_VEC");
}

void ToHorizon::process(){
    Vector3D<float> data;
    float yaw;
    yaw_port->read(yaw);
    Rot.setEulerYPR(-yaw, 0.0, 0.0);
    inp_vec_port->read(data);
    auto rotated_vec =  Rot*tf2::Vector3(data.x, data.y, data.z);
    out_vec_port->write(Vector3D<float>((float)rotated_vec.x(), (float)rotated_vec.y(), (float)rotated_vec.z()));
}

}