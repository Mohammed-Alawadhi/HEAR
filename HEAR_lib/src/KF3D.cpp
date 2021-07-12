#include "HEAR_control/KF3D.hpp"

#include <iostream>

namespace HEAR{

KF3D::KF3D(int b_uid, float dt) : Block(BLOCK_ID::KF, b_uid), _dt(dt) {
    u_gyro_port = createInputPort<Vector3D<float>>(IP::GYRO, "RAW_GYRO");
    u_acc_port = createInputPort<Vector3D<float>>(IP::ACC, "RAW_ACC");
    ang_meas_port = createInputPort<Vector3D<float>>(IP::ANGLES, "ANG_MEASUREMENTS");
    pos_meas_port = createInputPort<Vector3D<float>>(IP::POS, "POS_MEASUREMENT");
    predicted_pos = createOutputPort<Vector3D<float>>(OP::PRED_POS, "PREDICTED_POS");
    predicted_vel = createOutputPort<Vector3D<float>>(OP::PRED_VEL, "PREDCITED_VEL");
    predicted_angles = createOutputPort<Vector3D<float>>(OP::PRED_ANG, "PREDICTED_ANG");
    //predicted_acc_b = createOutputPort<Vector3D<float>>(OP::PRED_ACC_B, "PREDCITED_ACC_B");
    //predicted_gyro_b = createOutputPort<Vector3D<float>>(OP::PRED_GYRO_B, "PREDICTED_GYRO_B");
    reset();
}

void KF3D::reset() {
    _x.Zero();
    _x(6, 0) = 1;
    _P.Identity();
    _P = _P * 0.1;
    assert(_x.rows() == 16 && _x.cols() == 1);
    assert(_P.rows() == 16 && _P.cols() == 16);
    _H_ang.Zero(); _H_ang(0,6) = 1; _H_ang(1,7) = 1; _H_ang(2,8) = 1; _H_ang(3,9) = 1; 
    _H_pos.Zero(); _H_pos(0,0) = 1; _H_pos(1,1) = 1; _H_pos(2,2) = 1;
    _Q.diagonal() << 0.1893, 0.2238, 1.5781, 0.0097, 0.0133, 0.000106;
    _R_pos << 0.0000000339, 0.0000000875, 0.0000001986;
    _R_ang << 0.001, 0.001, 0.001, 0.001;
}

void KF3D::update(UpdateMsg* u_msg){

}
void KF3D::process(){
    predict();
    correct();
    publish();

}

void KF3D::predict(){
    u_gyro_port->read(_raw_gyro);
    u_acc_port->read(_raw_acc);
    predictAngle();
    predictVelocity();
    predictPosition();
    updatePredictionCoveriance();
    _P = _Fx * _P * _Fx.transpose() + _FQ * _Q *_FQ.transpose() + _Qtune;
}

void KF3D::predictAngle() {
    _omega(_raw_gyro.x- _x(13,0), _raw_gyro.y-_x(14,0), _raw_gyro.z-_x(15,0));
    //std::cout << "_omega: " << _omega.x << " " << _omega.y << " " << _omega.z << "\n";
    Quaternion4D<float> q_dot(0, _omega.x, _omega.y, _omega.z);
    _pred_ang(_x(6,0), _x(7,0), _x(8,0), _x(9,0));
    //std::cout << "_pred_ang: " << _pred_ang.w << " " << _pred_ang.x << " " << _pred_ang.y << " " << _pred_ang.z << "\n";
    q_dot = q_dot * _dt;
    _pred_ang = _pred_ang + (q_dot);
    _pred_ang.normalize();
    _x(6,0) = _pred_ang.w, _x(7,0) = _pred_ang.x, _x(8,0) = _pred_ang.y, _x(9,0) = _pred_ang.z;
};
void KF3D::predictVelocity() {
    _R.setRotation(tf2::Quaternion(_pred_ang.x, _pred_ang.y, _pred_ang.z, _pred_ang.w));
    float t2 = -_x(10,0);
    float t3 = -_x(11,0);
    float t4 = -_x(12,0);
    float t5 = _raw_acc.x+t2;
    float t6 = _raw_acc.y+t3;
    float t7 = _raw_acc.z+t4;
    _acc_inertial(_R[1].getX()*t5+_R[1].getY()*t6+_R[1].getZ()*t7, _R[2].getX()*t5+_R[2].getY()*t6+_R[2].getZ()*t7, _R[3].getX()*t5+_R[3].getY()*t6+_R[3].getZ()*t7-9.78909);
    _x(3,0) = _x(3,0)+_acc_inertial.x*_dt;
    _x(4,0) = _x(4,0)+_acc_inertial.y*_dt;
    _x(5,0) = _x(5,0)+_acc_inertial.z*_dt;
};
void KF3D::predictPosition() {
    float t2 = _dt*_dt;
    _x(0,0) = _x(0,0)+(_acc_inertial.x*t2)/2.0+_dt*_x(3,0);
    _x(1,0) = _x(1,0)+(_acc_inertial.y*t2)/2.0+_dt*_x(4,0);
    _x(2,0) = _x(2,0)+(_acc_inertial.z*t2)/2.0+_dt*_x(5,0);
};

void KF3D::updatePredictionCoveriance() {
    updatePredictionJacobian();
    updateProcessNoiseJacobian();
};
void KF3D::updatePredictionJacobian() {
    float q1 = _x(6,0), q2 =_x(7,0), q3 = _x(8,0), q4 =_x(9,0);
    float t2 = _dt*_dt;
    float t3 = q1*q1;
    float t4 = q2*q2;
    float t5 = q3*q3;
    float t6 = q4*q4;
    float t7 = q1*q2*2.0;
    float t8 = q1*q3*2.0;
    float t9 = q1*q4*2.0;
    float t10 = q2*q3*2.0;
    float t11 = q2*q4*2.0;
    float t12 = q3*q4*2.0;
    float t13 = -_x(10,0);
    float t14 = -_x(11,0);
    float t15 = -_x(12,0);
    float t22 = (_dt*_raw_gyro.x)/2.0;
    float t23 = (_dt*_raw_gyro.y)/2.0;
    float t24 = (_dt*_raw_gyro.z)/2.0;
    float t16 = -t10;
    float t17 = -t11;
    float t18 = -t12;
    float t19 = -t4;
    float t20 = -t5;
    float t21 = -t6;
    float t25 = _raw_acc.x+t13;
    float t26 = _raw_acc.y+t14;
    float t27 = _raw_acc.z+t15;
    float t28 = -t22;
    float t29 = -t23;
    float t30 = -t24;
    float t43 = t7+t12;
    float t44 = t8+t11;
    float t45 = t9+t10;
    float t31 = q1*t25*2.0;
    float t32 = q2*t25*2.0;
    float t33 = q1*t26*2.0;
    float t34 = q3*t25*2.0;
    float t35 = q2*t26*2.0;
    float t36 = q4*t25*2.0;
    float t37 = q1*t27*2.0;
    float t38 = q3*t26*2.0;
    float t39 = q2*t27*2.0;
    float t40 = q4*t26*2.0;
    float t41 = q3*t27*2.0;
    float t42 = q4*t27*2.0;
    float t49 = t7+t18;
    float t50 = t8+t17;
    float t51 = t9+t16;
    float t52 = t3+t6+t19+t20;
    float t53 = t3+t5+t19+t21;
    float t54 = t3+t4+t20+t21;
    float t46 = -t34;
    float t47 = -t39;
    float t48 = -t40;
    float t55 = t32+t38+t42;
    float t56 = t35+t37+t46;
    float t57 = t33+t36+t47;
    float t58 = t31+t41+t48;
    float t59 = _dt*t55;
    float t63 = (t2*t55)/2.0;
    float t60 = _dt*t56;
    float t61 = _dt*t57;
    float t62 = _dt*t58;
    float t64 = (t2*t56)/2.0;
    float t65 = (t2*t57)/2.0;
    float t66 = (t2*t58)/2.0;
    _Fx << 1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
           0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
           0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
           _dt,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
           0.0,_dt,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
           0.0,0.0,_dt,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,
           t66,t65,t64,t62,t61,t60,1.0,t22,t23,t24,0.0,0.0,0.0,0.0,0.0,0.0,
           t63,-t64,t65,t59,-t60,t61,t28,1.0,t30,t23,0.0,0.0,0.0,0.0,0.0,0.0,
           t64,t63,-t66,t60,t59,-t62,t29,t24,1.0,t28,0.0,0.0,0.0,0.0,0.0,0.0,
           -t65,t66,t63,-t61,t62,t59,t30,t29,t22,1.0,0.0,0.0,0.0,0.0,0.0,0.0,
           t2*t54*(-1.0/2.0),t2*t45*(-1.0/2.0),(t2*t50)/2.0,-_dt*t54,-_dt*t45,_dt*t50,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,
           (t2*t51)/2.0,t2*t53*(-1.0/2.0),t2*t43*(-1.0/2.0),_dt*t51,-_dt*t53,-_dt*t43,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,
           t2*t44*(-1.0/2.0),(t2*t49)/2.0,t2*t52*(-1.0/2.0),-_dt*t44,_dt*t49,-_dt*t52,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,
           0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,
           0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,
           0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0;
    assert((_Fx.rows() == _Fx.cols()) && (_Fx.rows() == 16));
};
void KF3D::updateProcessNoiseJacobian() {
    float q1 = _x(6,0), q2 =_x(7,0), q3 = _x(8,0), q4 =_x(9,0);
    float t2 = _dt*_dt;
    float t3 = q1*q1;
    float t4 = q2*q1;
    float t5 = q3*q3;
    float t6 = q4*q4;
    float t7 = q1*q2*2.0;
    float t8 = q1*q3*2.0;
    float t9 = q1*q4*2.0;
    float t10 = q2*q3*2.0;
    float t11 = q2*q4*2.0;
    float t12 = q3*q4*2.0;
    float t13 = -t10;
    float t14 = -t11;
    float t15 = -t12;
    float t16 = -t4;
    float t17 = -t5;
    float t18 = -t6;
    float t19 = t7+t12;
    float t20 = t8+t11;
    float t21 = t9+t10;
    float t22 = t7+t15;
    float t23 = t8+t14;
    float t24 = t9+t13;
    float t25 = t3+t6+t16+t17;
    float t26 = t3+t5+t16+t18;
    float t27 = t3+t4+t17+t18;
    _FQ << (t2*t27)/2.0,(t2*t21)/2.0,t2*t23*(-1.0/2.0),_dt*t27,_dt*t21,-_dt*t23,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,t2*t24*(-1.0/2.0),(t2*t26)/2.0,
            (t2*t19)/2.0,-_dt*t24,_dt*t26,_dt*t19,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,(t2*t20)/2.0,t2*t22*(-1.0/2.0),(t2*t25)/2.0,_dt*t20,
            -_dt*t22,_dt*t25,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0,
            0.0,0.0,0.0,0.0,0.0,0.0;
    assert((_FQ.rows() == 16) && (_FQ.cols() == 6));
};

void KF3D::correct() {
    Eigen::MatrixXf H, K, z, S, R, I_KH;
    H.conservativeResize(Eigen::NoChange_t::NoChange, 16);
    z.conservativeResize(Eigen::NoChange_t::NoChange, 1);
    Vector3D<float> _new_pos;
    Vector3D<float> _new_eul;
    tf2::Quaternion _new_ang;
    pos_meas_port->read(_new_pos);
    ang_meas_port->read(_new_eul);
    if(_new_pos != _meas_pos) {
        _meas_pos = _new_pos;
        H.conservativeResize(H.rows() + 3, Eigen::NoChange_t::NoChange);
        H.row(H.rows() - 3) << _H_pos;
        z.conservativeResize(z.rows()+3, Eigen::NoChange_t::NoChange);
        z.row(z.rows() - 3) << _new_pos.x, _new_pos.y, _new_pos.z;
        R.conservativeResize(R.rows() + 3, R.cols() + 3);
        R.block<3, 3>(R.rows()-3 , R.cols()-3) << _R_pos;
    }
    if(_new_eul != _meas_ang) {
        // _meas_ang = _new_eul;
        // _new_ang.setRPY(_new_eul.x, _new_eul.y, _new_eul.z);
        // H.conservativeResize(H.rows() + 4, Eigen::NoChange_t::NoChange);
        // H.row(H.rows() - 4) << _H_ang;
        // z.conservativeResize(z.rows()+4, Eigen::NoChange_t::NoChange);
        // z.row(z.rows() - 4) << _new_ang.getW(), _new_ang.getX(), _new_ang.getY(), _new_ang.getZ();
        // R.conservativeResize(R.rows() + 4, R.cols() + 4);
        // R.block<4, 4>(R.rows()-4 , R.cols()-4) = _R_ang.asDiagonal();
    }
    if(H.rows() > 0) {
        //std:: cout << H.rows() << " " << 
        S = H * _P * H.transpose();
        K = _P * H.transpose() * S.inverse();
        _x = _x + K * (z - H * _x);
        _pred_ang(_x(6,0), _x(7,0), _x(8,0), _x(9,0));
        _pred_ang.normalize();
        _x(6,0) = _pred_ang.w, _x(7,0) = _pred_ang.x, _x(8,0) = _pred_ang.y, _x(9,0) = _pred_ang.z;
        I_KH = Eigen::MatrixXf::Identity(_P.rows(), _P.cols()) - K*H;
        _P = I_KH * _P * I_KH.transpose() + K * R * K.transpose();
    }
}

void KF3D::publish() {
    tf2::Matrix3x3 _pred_rot; _pred_rot.setRotation(tf2::Quaternion(_pred_ang.x, _pred_ang.y, _pred_ang.z, _pred_ang.w));
    double yaw, pitch, roll;
    _pred_rot.getEulerYPR(yaw, pitch, roll);
    predicted_angles->write(Vector3D<float>(roll, pitch, yaw));

    predicted_vel->write(Vector3D<float>(_x(3, 0), _x(4, 0), _x(5, 0)));
    
    predicted_pos->write(Vector3D<float>(_x(0, 0), _x(1, 0), _x(2, 0)));
}

}