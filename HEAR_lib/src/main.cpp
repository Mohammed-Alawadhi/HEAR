

#include "HEAR_control/PID_Block.hpp"
#include "HEAR_control/Differentiator.hpp"
#include "HEAR_control/BWFilter.hpp"
#include "HEAR_control/FromHorizon.hpp"
#include "HEAR_control/ToHorizon.hpp"
#include "HEAR_control/FbLinearizer.hpp"
#include "HEAR_control/Demux3.hpp"
#include "HEAR_control/Eul2Rot.hpp"
#include "HEAR_control/Saturation.hpp"
#include "HEAR_control/HexaActuationSystem.hpp"
#include "HEAR_control/Sum.hpp"
#include "HEAR_control/Mux3.hpp"

#include "HEAR_ROS/ROSUnit_FloatPub.hpp"
#include "HEAR_ROS/ROSUnit_FloatSub.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"
#include "HEAR_ROS/ROSUnit_ResetSrv.hpp"
#include "HEAR_ROS/ROSUnit_FloatArrPub.hpp"
#include "HEAR_ROS/ROSUnit_PointPub.hpp"

#include "HEAR_core/System.hpp"
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/ExternalTrigger.hpp"
#include "HEAR_control/BWFilter.hpp"
#include "HEAR_core/Vector3D.hpp"
#include "ros/ros.h"
#include <iostream>

using namespace HEAR;

const int FREQ_OUTER = 120;
const int FREQ_INNER = 200;

int main(int argc, char** argv) {
    ros::init(argc, argv, "tesing_node");
    ros::NodeHandle nh;
//    ros::Rate rate(FREQ);

    ROSUnitFloatSub waypoint_x(nh);
    ROSUnitFloatSub waypoint_y(nh);
    ROSUnitFloatSub waypoint_z(nh);
    ROSUnitFloatSub waypoint_yaw(nh);

    ROSUnit_PoseProvider providers(nh);

    auto wp_x_port = waypoint_x.registerSubscriber("waypoint_reference/x");
    auto wp_y_port = waypoint_y.registerSubscriber("waypoint_reference/y");
    auto wp_z_port = waypoint_z.registerSubscriber("waypoint_reference/z");
    auto wp_yaw_port = waypoint_yaw.registerSubscriber("waypoint_reference/yaw");

    auto opti_port = providers.registerOptiPose("/Robot_1/pose");
    auto imu_ori_port = providers.registerImuOri("filter/quaternion");
    auto imu_rate_port = providers.registerImuAngularRate("imu/angular_velocity");

    // Setting up the outer loop

    auto outer_sys = new System(FREQ_OUTER);
    //creating external input ports
    auto pos_port_idx = outer_sys->createExternalInputPort<Vector3D<float>>(TYPE::Float3, "opti_pos_port");
    outer_sys->connectToExternalInput(pos_port_idx, opti_port[0]);
    auto opti_ori_port_idx = outer_sys->createExternalInputPort<Vector3D<float>>(TYPE::Float3, "opti_ori_port");
    outer_sys->connectToExternalInput(opti_ori_port_idx, opti_port[1]);
    auto wp_x_inp_idx = outer_sys->createExternalInputPort<float>(TYPE::Float, "wp_x_port");
    outer_sys->connectToExternalInput(wp_x_inp_idx, wp_x_port);
    auto wp_y_inp_idx = outer_sys->createExternalInputPort<float>(TYPE::Float, "wp_y_port");
    outer_sys->connectToExternalInput(wp_y_inp_idx, wp_y_port);
    auto wp_z_inp_idx = outer_sys->createExternalInputPort<float>(TYPE::Float, "wp_z_port");
    outer_sys->connectToExternalInput(wp_z_inp_idx, wp_z_port);
    auto wp_yaw_inp_idx = outer_sys->createExternalInputPort<float>(TYPE::Float, "wp_yaw_port");
    outer_sys->connectToExternalInput(wp_yaw_inp_idx, wp_yaw_port);
    
    auto diff_pos_block = new Differentiator<Vector3D<float>>(TYPE::Float3);
    auto dif_pos_idx = outer_sys->addBlock(diff_pos_block, "pos_derivative");
    outer_sys->connectToExternalInput<Vector3D<float>>(pos_port_idx, dif_pos_idx, Differentiator<Vector3D<float>>::IP::INPUT);
    auto demux_opti_ori = new Demux3();
    auto op_ori_demux_idx = outer_sys->addBlock(demux_opti_ori, "demux_opti_ori");
    outer_sys->connectToExternalInput<Vector3D<float>>(opti_ori_port_idx, op_ori_demux_idx, Demux3::IP::INPUT);
    auto to_horizon_pos = new ToHorizon();
    auto to_pos_h_idx = outer_sys->addBlock(to_horizon_pos, "to_horizon_pos");
    outer_sys->connect<float>(op_ori_demux_idx, Demux3::OP::Z, to_pos_h_idx, ToHorizon::IP::YAW);
    outer_sys->connectToExternalInput<Vector3D<float>>(pos_port_idx, to_pos_h_idx, ToHorizon::IP::INP_VEC);
    auto to_horizon_vel = new ToHorizon();
    auto to_vel_h_idx = outer_sys->addBlock(to_horizon_vel, "to_horizon_vel");
    outer_sys->connect<float>(op_ori_demux_idx, Demux3::OP::Z, to_vel_h_idx, ToHorizon::IP::YAW);
    outer_sys->connect<float>(dif_pos_idx, Differentiator<Vector3D<float>>::OP::OUTPUT, to_vel_h_idx, ToHorizon::IP::INP_VEC);
    auto demux_pos = new Demux3();
    auto demux_pos_idx = outer_sys->addBlock(demux_pos, "pos_demux");
    outer_sys->connect<Vector3D<float>>(to_pos_h_idx, ToHorizon::OP::OUT_VEC, demux_pos_idx, Demux3::IP::INPUT);
    auto demux_vel = new Demux3();
    auto demux_vel_idx = outer_sys->addBlock(demux_vel, "vel_demux");
    outer_sys->connect<Vector3D<float>>(to_vel_h_idx, ToHorizon::OP::OUT_VEC, demux_vel_idx, Demux3::IP::INPUT);
    auto sum_ref_x = new Sum(Sum::OPERATION::SUB);
    auto sum_ref_x_idx = outer_sys->addBlock(sum_ref_x, "sum_ref_x");
    outer_sys->connectToExternalInput<float>(wp_x_inp_idx, sum_ref_x_idx, Sum::IP::OPERAND1);
    outer_sys->connect<float>(demux_pos_idx, Demux3::OP::X, sum_ref_x_idx, Sum::IP::OPERAND2);
    auto sum_ref_y = new Sum(Sum::OPERATION::SUB);
    auto sum_ref_y_idx = outer_sys->addBlock(sum_ref_y, "sum_ref_y");
    outer_sys->connectToExternalInput<float>(wp_y_inp_idx, sum_ref_y_idx, Sum::IP::OPERAND1);
    outer_sys->connect<float>(demux_pos_idx, Demux3::OP::Y, sum_ref_y_idx, Sum::IP::OPERAND2);
    auto sum_ref_z = new Sum(Sum::OPERATION::SUB);
    auto sum_ref_z_idx = outer_sys->addBlock(sum_ref_z, "sum_ref_z");
    outer_sys->connectToExternalInput<float>(wp_z_inp_idx, sum_ref_z_idx, Sum::IP::OPERAND1);
    outer_sys->connect<float>(demux_pos_idx, Demux3::OP::Z, sum_ref_z_idx, Sum::IP::OPERAND2);
    auto PID_x = new PID_Block(PID_ID::PID_X);
    auto pid_x_idx = outer_sys->addBlock(PID_x, "PID_x");
    outer_sys->connect<float>(sum_ref_x_idx, Sum::OP::OUTPUT, pid_x_idx, PID_Block::IP::ERROR);
    outer_sys->connect<float>(demux_vel_idx, Demux3::OP::X, pid_x_idx, PID_Block::IP::PV_DOT);
    auto PID_y = new PID_Block(PID_ID::PID_Y);
    auto pid_y_idx = outer_sys->addBlock(PID_y, "PID_y");
    outer_sys->connect<float>(sum_ref_y_idx, Sum::OP::OUTPUT, pid_y_idx, PID_Block::IP::ERROR);
    outer_sys->connect<float>(demux_vel_idx, Demux3::OP::Y, pid_y_idx, PID_Block::IP::PV_DOT);
    auto PID_z = new PID_Block(PID_ID::PID_Z);
    auto pid_z_idx = outer_sys->addBlock(PID_z, "PID_z");
    outer_sys->connect<float>(sum_ref_z_idx, Sum::OP::OUTPUT, pid_z_idx, PID_Block::IP::ERROR);
    outer_sys->connect<float>(demux_vel_idx, Demux3::OP::Z, pid_z_idx, PID_Block::IP::PV_DOT);    
    auto mux_fh_des = new Mux3();
    auto mux_fh_des_idx = outer_sys->addBlock(mux_fh_des, "mux_fh_des");
    outer_sys->connect<float>(pid_x_idx, PID_Block::OP::COMMAND, mux_fh_des_idx, Mux3::IP::X);
    outer_sys->connect<float>(pid_y_idx, PID_Block::OP::COMMAND, mux_fh_des_idx, Mux3::IP::Y);
    outer_sys->connect<float>(pid_z_idx, PID_Block::OP::COMMAND, mux_fh_des_idx, Mux3::IP::Z);
    auto fh2fi = new FromHorizon();
    auto fh2fi_idx = outer_sys->addBlock(fh2fi, "fh2fi");
    outer_sys->connect<Vector3D<float>>(mux_fh_des_idx, Mux3::OP::OUTPUT, fh2fi_idx, FromHorizon::IP::INP_VEC);
    auto force2rot = new FbLinearizer::Force2Rot();
    auto f2r_idx = outer_sys->addBlock(force2rot, "f2rot");
    outer_sys->connect<Vector3D<float>>(fh2fi_idx, FromHorizon::OP::OUT_VEC, f2r_idx, FbLinearizer::Force2Rot::IP::FORCE_I_DES);
    outer_sys->connectToExternalInput<float>(wp_yaw_inp_idx, f2r_idx, FbLinearizer::Force2Rot::IP::YAW_REF);

    auto pos_h_port_idx = outer_sys->createExternalOutputPort<Vector3D<float>>(TYPE::Float3, "pos_horizon_port");
    outer_sys->connectToExternalOutput<Vector3D<float>>(to_pos_h_idx, ToHorizon::OP::OUT_VEC, pos_h_port_idx);
    auto des_rot_port_idx = outer_sys->createExternalOutputPort<tf2::Matrix3x3>(TYPE::RotMat, "des_rot_b_port");
    outer_sys->connectToExternalOutput<tf2::Matrix3x3>(f2r_idx, FbLinearizer::Force2Rot::OP::ROT_DES, des_rot_port_idx);
    auto pub_pos_h = new ROSUnitPointPub(nh);
    outer_sys->connectToExternalOutput<Vector3D<float>>(pub_pos_h->registerPublisher("/pos_horizon"), pos_h_port_idx);
    outer_sys->addPub(pub_pos_h);
    auto fh_des_port_idx = outer_sys->createExternalOutputPort<Vector3D<float>>(TYPE::Float3, "fh_des_port");
    outer_sys->connectToExternalOutput<Vector3D<float>>(mux_fh_des_idx, Mux3::OP::OUTPUT, fh_des_port_idx);
    auto pub_fh_des = new ROSUnitPointPub(nh);
    outer_sys->connectToExternalOutput<Vector3D<float>>(pub_fh_des->registerPublisher("/fh_des"), fh_des_port_idx);
    outer_sys->addPub(pub_fh_des);
    auto fi_des_port_idx = outer_sys->createExternalOutputPort<Vector3D<float>>(TYPE::Float3, "fi_des_port");
    outer_sys->connectToExternalOutput<Vector3D<float>>(fh2fi_idx, FromHorizon::OP::OUT_VEC, fi_des_port_idx);
    auto pub_fi_des = new ROSUnitPointPub(nh);
    outer_sys->connectToExternalOutput<Vector3D<float>>(pub_fi_des->registerPublisher("/fi_des"), fi_des_port_idx);
    outer_sys->addPub(pub_fi_des);

    //end of outer loop system

    // Setting up inner loop
    auto inner_sys = new System(FREQ_INNER);
    auto rb_des_idx = inner_sys->createExternalInputPort<tf2::Matrix3x3>(TYPE::RotMat, "RB_des_port");
    inner_sys->connectToExternalInput(rb_des_idx, outer_sys->getExternalOutputPort<tf2::Matrix3x3>(des_rot_port_idx));
    auto fi_des_idx = inner_sys->createExternalInputPort<Vector3D<float>>(TYPE::Float3, "fi_des_port");
    inner_sys->connectToExternalInput(fi_des_idx, outer_sys->getExternalOutputPort<Vector3D<float>>(fi_des_idx));
    auto opti_ori_idx = inner_sys->createExternalInputPort<Vector3D<float>>(TYPE::Float3, "opti_ori");
    inner_sys->connectToExternalInput(opti_ori_idx, opti_port[1]); 
    auto ori_port_idx = inner_sys->createExternalInputPort<Vector3D<float>>(TYPE::Float3, "ori_port");
    inner_sys->connectToExternalInput(ori_port_idx, imu_ori_port);
    auto angle_rate_port_idx = inner_sys->createExternalInputPort<Vector3D<float>>(TYPE::Float3, "angle_rate_port");
    inner_sys->connectToExternalInput(angle_rate_port_idx, imu_rate_port);

    auto bw_filt = new BWFilter2<Vector3D<float>>(BWFilt2_coeff::coeff_N200C50, TYPE::Float3);
    auto bw_filt_idx = inner_sys->addBlock(bw_filt, "bw_filt");
    inner_sys->connectToExternalInput<Vector3D<float>>(angle_rate_port_idx, bw_filt_idx, bw_filt->INPUT);
    auto demux_angle_rate = new Demux3();
    auto demux_angle_rate_idx = inner_sys->addBlock(demux_angle_rate, "demux_angle_rate");
    inner_sys->connect<Vector3D<float>>(bw_filt_idx, bw_filt->OUTPUT, demux_angle_rate_idx, Demux3::IP::INPUT);
    auto demux_imu = new Demux3();
    auto demux_imu_idx = inner_sys->addBlock(demux_imu, "demux_imu");
    inner_sys->connectToExternalInput<Vector3D<float>>(ori_port_idx, demux_imu_idx, Demux3::IP::INPUT);
    auto demux_opti = new Demux3();
    auto demux_opti_idx = inner_sys->addBlock(demux_imu, "demux_opti");
    inner_sys->connectToExternalInput<Vector3D<float>>(opti_ori_idx, demux_opti_idx, Demux3::IP::INPUT);
    auto mux_eul = new Mux3();
    auto mux_eul_idx = inner_sys->addBlock(mux_eul, "mux_rpy");
    inner_sys->connect<float>(demux_imu_idx, Demux3::OP::X, mux_eul_idx, Mux3::IP::X);
    inner_sys->connect<float>(demux_imu_idx, Demux3::OP::Y, mux_eul_idx, Mux3::IP::Y);
    inner_sys->connect<float>(demux_opti_idx, Demux3::OP::Z, mux_eul_idx, Mux3::IP::Z);
    auto eul2rot = new Eul2Rot();
    auto eul2rot_idx = inner_sys->addBlock(eul2rot, "eul2rot");
    inner_sys->connect<Vector3D<float>>(mux_eul_idx, Mux3::OP::OUTPUT, eul2rot_idx, Eul2Rot::IP::EUL_ANGLES);
    auto rot2angle = new FbLinearizer::RotDiff2Rod();
    auto rot2angle_idx = inner_sys->addBlock(rot2angle, "rdiff2rod");
    inner_sys->connect<tf2::Matrix3x3>(eul2rot_idx, Eul2Rot::OP::ROT_MAT, rot2angle_idx, FbLinearizer::RotDiff2Rod::IP::R_I_B);
    inner_sys->connectToExternalInput<tf2::Matrix3x3>(rb_des_idx, rot2angle_idx, FbLinearizer::RotDiff2Rod::IP::R_BDES_I);
    inner_sys->connectToExternalInput<Vector3D<float>>(fi_des_idx, rot2angle_idx, FbLinearizer::RotDiff2Rod::IP::F_IDES);
    auto demux_angle_err = new Demux3();
    auto demux_angle_err_idx = inner_sys->addBlock(demux_angle_err, "demux_angle_err");
    inner_sys->connect<Vector3D<float>>(rot2angle_idx, FbLinearizer::RotDiff2Rod::OP::ROD_ANGLES, demux_angle_err_idx, Demux3::IP::INPUT);
    auto PID_roll = new PID_Block(PID_ID::PID_ROLL);
    auto pid_roll_idx = inner_sys->addBlock(PID_roll, "PID_roll");
    inner_sys->connect<float>(demux_angle_err_idx, Demux3::OP::X, pid_roll_idx, PID_Block::IP::ERROR);
    inner_sys->connect<float>(demux_angle_rate_idx, Demux3::OP::X, pid_roll_idx, PID_Block::IP::PV_DOT);
    auto PID_pitch = new PID_Block(PID_ID::PID_PITCH);
    auto pid_pitch_idx = inner_sys->addBlock(PID_pitch, "PID_pitch");
    inner_sys->connect<float>(demux_angle_err_idx, Demux3::OP::Y, pid_pitch_idx, PID_Block::IP::ERROR);
    inner_sys->connect<float>(demux_angle_rate_idx, Demux3::OP::Y, pid_pitch_idx, PID_Block::IP::PV_DOT);
    auto PID_yaw = new PID_Block(PID_ID::PID_YAW);
    auto pid_yaw_idx = inner_sys->addBlock(PID_yaw, "PID_yaw");
    inner_sys->connect<float>(demux_angle_err_idx, Demux3::OP::Z, pid_yaw_idx, PID_Block::IP::ERROR);
    auto sum_ref_yaw = new Sum(Sum::OPERATION::SUB);
    auto sum_ref_yaw_idx = inner_sys->addBlock(sum_ref_yaw, "sum_ref_yaw");
    inner_sys->connect<float>(pid_yaw_idx, PID_Block::OP::COMMAND, sum_ref_yaw_idx, Sum::IP::OPERAND1);
    inner_sys->connect<float>(demux_angle_rate_idx, Demux3::OP::Z,  sum_ref_yaw_idx, Sum::IP::OPERAND2);
    auto PID_yaw_rate = new PID_Block(PID_ID::PID_YAW_RATE);
    auto pid_yaw_rate_idx = inner_sys->addBlock(PID_yaw, "PID_yaw_rate");
    inner_sys->connect<float>(sum_ref_yaw_idx, Sum::OP::OUTPUT, pid_yaw_rate_idx, PID_Block::IP::ERROR);
    auto mux_angle_u = new Mux3();
    auto mux_angle_u_idx = inner_sys->addBlock(mux_angle_u, "mux_angle_u");
    inner_sys->connect<float>(pid_roll_idx, PID_Block::OP::COMMAND, mux_angle_u_idx, Mux3::IP::X);
    inner_sys->connect<float>(pid_pitch_idx, PID_Block::OP::COMMAND, mux_angle_u_idx, Mux3::IP::Y);
    inner_sys->connect<float>(pid_yaw_rate_idx, PID_Block::OP::COMMAND, mux_angle_u_idx, Mux3::IP::Z);

    auto angle_u_port_idx = inner_sys->createExternalOutputPort<Vector3D<float>>(TYPE::Float3, "angle_u_port");
    inner_sys->connectToExternalOutput<Vector3D<float>>(mux_angle_u_idx, Mux3::OP::OUTPUT, angle_u_port_idx);
    auto uz_port_idx = inner_sys->createExternalOutputPort<float>(TYPE::Float, "uz_port");
    inner_sys->connectToExternalOutput<float>(rot2angle_idx, FbLinearizer::RotDiff2Rod::OP::THRUST, uz_port_idx);

    // TODO: add publishers

    // end of inner loop system

    //setting up actuation




    while(ros::ok()){
        ros::spin();
    }

}
