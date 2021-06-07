#include "HEAR_control/FbLinearizer.hpp"

namespace HEAR{

namespace FbLinearizer{

Force2Rot::Force2Rot() : Block(BLOCK_ID::FORCE2ROT){
    yaw_ref_port = createInputPort<float>(IP::YAW_REF, TYPE::Float, "YAW_REF");
    force_i_des_port = createInputPort<Vector3D<float>>(IP::FORCE_I_DES, TYPE::Float3, "FORCE_I_DES");
    rot_des_port = createOutputPort<tf2::Matrix3x3>(OP::ROT_DES, TYPE::RotMat, "ROT_DES");

}

void Force2Rot::process(){
    Vector3D<float> F_I_des;
    float yaw_ref;
    yaw_ref_port->read(yaw_ref);
    force_i_des_port->read(F_I_des);
    auto z_B_des = F_I_des.normalized();
    auto y_aux = F_I_des.cross(Vector3D<float>(cos(yaw_ref), sin(yaw_ref), 0.0));
    auto y_B_des = y_aux.normalized();
    auto x_B_des = y_B_des.cross(z_B_des); //TODO: check if it is z_B or z_B_des
    auto R_B_des_I = tf2::Matrix3x3(x_B_des.x, x_B_des.y, x_B_des.z,
                                    y_B_des.x, y_B_des.y, y_B_des.z,
                                    z_B_des.x, z_B_des.y, z_B_des.z);
    rot_des_port->write(R_B_des_I);                                
}

RotDiff2Rod::RotDiff2Rod() : Block(BLOCK_ID::ROTDIFF2ROD){
    r_i_b_port = createInputPort<tf2::Matrix3x3>(IP::R_I_B, TYPE::RotMat, "R_I_B");
    f_ides_port = createInputPort<Vector3D<float>>(IP::F_IDES, TYPE::Float3, "F_IDES");
    r_bdes_i_port = createInputPort<tf2::Matrix3x3>(IP::R_BDES_I, TYPE::RotMat, "R_BDES_I");
    angles_port = createOutputPort<Vector3D<float>>(OP::ROD_ANGLES, TYPE::Float3, "ROD_ANGLES");
    thrust_port = createOutputPort<float>(OP::THRUST, TYPE::Float, "THRUST");

}

void RotDiff2Rod::process(){
    tf2::Matrix3x3 R_I_B, R_B_des_I;
    Vector3D<float> F_I_des;
    r_i_b_port->read(R_I_B);
    r_bdes_i_port->read(R_B_des_I);
    f_ides_port->read(F_I_des);
    double r, p, y;
    R_I_B.getRPY(r, p, y);
    std::cout << "R_I_B : "
                << r << " " << p << " " << y << std::endl;
    R_B_des_I.getRPY(r, p, y);
    std::cout << "R_B_des_I :\n"
                << r << " " << p << " " << y << std::endl;
    
    auto R_B_B_des = R_I_B.transposeTimes(R_B_des_I.transpose());
    tf2::Quaternion quat;
    R_B_B_des.getRotation(quat);
    auto angle = quat.getAngle();
    auto err_angles = (angle <= M_PI? angle: angle - 2*M_PI)*quat.getAxis();

    auto u_z = (R_I_B.transpose()*tf2::Vector3(F_I_des.x, F_I_des.y, F_I_des.z)).z();
    std::cout << u_z <<std::endl;
    angles_port->write(Vector3D<float>((float)err_angles.x(), (float)err_angles.y(), (float)err_angles.z()));
    thrust_port->write(u_z);
}
}
}