#ifndef KF3D_HPP
#define KF3D_HPP

//#include "eigen3/Eigen/Core"
//#include "eigen3/Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "HEAR_core/Block.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "HEAR_core/Quaternion4D.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <cassert>
#include <ros/ros.h>
#include <fstream> //TODO: delete

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
    Vector3D<float> _raw_gyro, _old_gyro, _raw_acc, _meas_pos, _omega, _acc_inertial, _meas_ang;
    Quaternion4D<float> _pred_ang;
    Vector3D<float> _new_pos;
    Vector3D<float> _new_eul;
    tf2::Quaternion _new_ang;
    bool initialized = false; //TODO: delete
    std::ofstream outdata; //TODO: delete
    ros::Time _prev_time;


    Eigen::Matrix<float, 13, 13> _Fx, _P, _Qtune;
    Eigen::Matrix<float, 13, 1> _x;
    Eigen::Matrix<float, 6,  13> _FQ;
    Eigen::DiagonalMatrix<float, 6> _Q;
    Eigen::Matrix<float, 3, 3> _R_pos;
    Eigen::Matrix<float, 4, 4> _R_ang;
    Eigen::Matrix<float, 3, 13> _H_pos;
    Eigen::Matrix<float, 4, 13> _H_ang;

    void readInputs();
    void predict();
    void correct();
    void doCorrectionMath(const Eigen::Ref<const Eigen::MatrixXf> &, const Eigen::Ref<const Eigen::MatrixXf> &, const Eigen::Ref<const Eigen::MatrixXf> &);
    void predictAngle();
    void calcInertialAcceleration();
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
    ~KF3D();
    void process();

};

}

#endif