#ifndef KF3D_HPP
#define KF3D_HPP

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Dense"
#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/Quaternion4D.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <cassert>

namespace HEAR{

class KF3D : public Block{
private:
    InputPort<Vector3D<float>>* u_gyro_port;
    InputPort<Vector3D<float>>* u_acc_port;
    InputPort<Vector3D<float>>* ang_meas_port;
    InputPort<Vector3D<float>>* pos_meas_port;
    OutputPort<Vector3D<float>>* predicted_pos;
    OutputPort<Vector3D<float>>* predicted_vel;
    OutputPort<Vector3D<float>>* predicted_angles;
    //OutputPort<Vector3D<float>>* predicted_acc_b;
    //OutputPort<Vector3D<float>>* predicted_gyro_b;
    float _dt;
    Vector3D<float> _raw_gyro, _raw_acc, _meas_pos, _omega, _acc_inertial, _meas_ang;
    Quaternion4D<float> _pred_ang;
    tf2::Matrix3x3 _R;

    Eigen::Matrix<float, 16, 16> _Fx, _P, _Qtune;
    Eigen::Matrix<float, 16, 1> _x;
    Eigen::Matrix<float, 16, 6> _FQ;
    Eigen::DiagonalMatrix<float, 6> _Q;
    Eigen::Vector3f _R_pos;
    Eigen::Vector4f _R_ang;
    Eigen::Matrix<float, 3, 16> _H_pos;
    Eigen::Matrix<float, 4, 16> _H_ang;

    void predict();
    void correct();
    void predictAngle();
    void predictVelocity();
    void predictPosition();
    void updatePredictionCoveriance();
    void updatePredictionJacobian();
    void updateProcessNoiseJacobian();
    void publish();

public:
    enum IP{GYRO, ACC, ANGLES, POS};
    enum OP{PRED_POS, PRED_VEL, PRED_ANG};//, PRED_ACC_B, PRED_GYRO_B};
    enum COEFF{};

    KF3D(int b_uid, float dt);
    void setCoeff(COEFF, float);
    void update(UpdateMsg* u_msg) override;
    void reset() override;
    ~KF3D(){}
    void process();

};

}

#endif