
#include "OuterSysNodelet.h"
#include <pluginlib/class_list_macros.h>



PLUGINLIB_EXPORT_CLASS(HEAR::OuterSysNodelet, nodelet::Nodelet)

namespace HEAR
{
    OuterSysNodelet::~OuterSysNodelet(){
        delete outer_sys;
    }
    void OuterSysNodelet::onInit(){
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        outer_sys = new RosSystem(nh, pnh, FREQUENCY, "OuterLoop");

        // creating Blocks
        auto diff_pos = outer_sys->createBlock(BLOCK_ID::DIFFERENTIATOR, "Pos_Derivative", TYPE::Float3);
        auto pos_filt = outer_sys->createBlock(BLOCK_ID::BW_FILT2, "Vel_Filt", TYPE::Float3);
        ((BWFilter2<Vector3D<float>>*)pos_filt)->setCoeff(BWFilt2_coeff::coeff_120Hz_2nd_butter_5hz);
        auto demux_ori = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_ori");
        auto to_horizon_pos = outer_sys->createBlock(BLOCK_ID::TOHORIZON, "ToHorizon_Pos");
        auto to_horizon_vel = outer_sys->createBlock(BLOCK_ID::TOHORIZON, "ToHorizon_Vel");
        auto pos_h_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Pos_H_Demux");
        auto vel_h_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Vel_H_Demux");
        auto sum_ref_x = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_x");
        auto sum_ref_y = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_y");
        auto sum_ref_z = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_z");
        auto pid_x = outer_sys->createBlock(BLOCK_ID::PID, "Pid_x"); ((PID_Block*)pid_x)->setPID_ID(PID_ID::PID_X);
        auto pid_y = outer_sys->createBlock(BLOCK_ID::PID, "Pid_y"); ((PID_Block*)pid_y)->setPID_ID(PID_ID::PID_Y);
        auto pid_z = outer_sys->createBlock(BLOCK_ID::PID, "Pid_z"); ((PID_Block*)pid_z)->setPID_ID(PID_ID::PID_Z);
        auto sat_x = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_x"); ((Saturation*)sat_x)->setClipValue(SAT_XY_VALUE); 
        auto sat_y = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_y"); ((Saturation*)sat_y)->setClipValue(SAT_XY_VALUE);
        auto sat_z = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_z"); ((Saturation*)sat_z)->setClipValue(SAT_Z_VALUE);
        auto mux_fh_des = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_Fh");
        auto fh2fi = outer_sys->createBlock(BLOCK_ID::FROMHORIZON, "Fh_to_Fi");
        auto f2rot = outer_sys->createBlock(BLOCK_ID::FORCE2ROT, "Fi_to_RotMat");
        auto rot2eul = outer_sys->createBlock(BLOCK_ID::ROT2EUL, "RotMat_to_Eul");
        auto demux_eul_des = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_EulDes");
        auto mux_eul_des = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_EulDes");
        auto bias_z = outer_sys->createBlock(BLOCK_ID::CONSTANT, "Bias_z", TYPE::Float); ((Constant<float>*)bias_z)->setValue(BIAS_Z_VALUE);
        auto sum_bias_z = outer_sys->createBlock(BLOCK_ID::SUM, "sum_bias_z"); ((Sum*)sum_bias_z)->setOperation(Sum::OPERATION::ADD);

        // connecting blocks
        providers = new ROSUnit_PoseProvider(nh);
        auto pos_port = outer_sys->createExternalInputPort<Vector3D<float>>("Pos_Port");
        auto ori_port = outer_sys->createExternalInputPort<Vector3D<float>>("Ori_Port");
        auto opti_port = providers->registerOptiPose("/Robot_1/pose");
        outer_sys->connectExternalInput(pos_port, opti_port[0]);
        outer_sys->connectExternalInput(ori_port, opti_port[1]);

        outer_sys->connectExternalInput(ori_port, demux_ori->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        outer_sys->connectExternalInput(pos_port, diff_pos->getInputPort<Vector3D<float>>(0));
        outer_sys->connect(diff_pos->getOutputPort<Vector3D<float>>(0), pos_filt->getInputPort<Vector3D<float>>(0));
        outer_sys->connectExternalInput(pos_port, to_horizon_pos->getInputPort<Vector3D<float>>(ToHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), to_horizon_pos->getInputPort<float>(ToHorizon::IP::YAW));
        outer_sys->connect(pos_filt->getOutputPort<Vector3D<float>>(0), to_horizon_vel->getInputPort<Vector3D<float>>(ToHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), to_horizon_vel->getInputPort<float>(ToHorizon::IP::YAW));
        outer_sys->connect(to_horizon_pos->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC), pos_h_demux->getInputPort<Vector3D<float>>(Demux3::INPUT));
        outer_sys->connect(to_horizon_vel->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC), vel_h_demux->getInputPort<Vector3D<float>>(Demux3::INPUT));
        
        outer_sys->createSub("/waypoint_reference/x", sum_ref_x->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::X), sum_ref_x->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_ref_x->getOutputPort<float>(Sum::OP::OUTPUT), pid_x->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::X), pid_x->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->connect(pid_x->getOutputPort<float>(PID_Block::OP::COMMAND), mux_fh_des->getInputPort<float>(Mux3::IP::X));

        outer_sys->createSub("/waypoint_reference/y", sum_ref_y->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::Y), sum_ref_y->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_ref_y->getOutputPort<float>(Sum::OP::OUTPUT), pid_y->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::Y), pid_y->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->connect(pid_y->getOutputPort<float>(PID_Block::OP::COMMAND), mux_fh_des->getInputPort<float>(Mux3::IP::Y));

        outer_sys->createSub("/waypoint_reference/z", sum_ref_z->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::Z), sum_ref_z->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_ref_z->getOutputPort<float>(Sum::OP::OUTPUT), pid_z->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::Z), pid_z->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->connect(pid_z->getOutputPort<float>(PID_Block::OP::COMMAND), sum_bias_z->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(bias_z->getOutputPort<float>(Constant<float>::OP::OUTPUT), sum_bias_z->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_bias_z->getOutputPort<float>(Sum::OP::OUTPUT), sat_z->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(sat_z->getOutputPort<float>(Saturation::OP::OUTPUT), mux_fh_des->getInputPort<float>(Mux3::IP::Z));

        outer_sys->connect(mux_fh_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT), fh2fi->getInputPort<Vector3D<float>>(FromHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), fh2fi->getInputPort<float>(FromHorizon::IP::YAW));

        outer_sys->connect(fh2fi->getOutputPort<Vector3D<float>>(FromHorizon::OP::OUT_VEC), f2rot->getInputPort<Vector3D<float>>(FbLinearizer::Force2Rot::IP::FORCE_I_DES));
        outer_sys->createSub("/waypoint_reference/yaw", f2rot->getInputPort<float>(FbLinearizer::Force2Rot::IP::YAW_REF));

        outer_sys->connect(f2rot->getOutputPort<tf2::Matrix3x3>(FbLinearizer::Force2Rot::OP::ROT_DES), rot2eul->getInputPort<tf2::Matrix3x3>(Rot2Eul::IP::ROT_MAT));
        outer_sys->connect(rot2eul->getOutputPort<Vector3D<float>>(Rot2Eul::OP::EUL_ANGLES), demux_eul_des->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::X), sat_x->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::Y), sat_y->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(sat_x->getOutputPort<float>(Saturation::OP::OUTPUT), mux_eul_des->getInputPort<float>(Mux3::IP::X));
        outer_sys->connect(sat_y->getOutputPort<float>(Saturation::OP::OUTPUT), mux_eul_des->getInputPort<float>(Mux3::IP::Y));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::Z), mux_eul_des->getInputPort<float>(Mux3::IP::Z));

        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/pos_horizon", to_horizon_pos->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/fh_des", mux_fh_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/fi_des", fh2fi->getOutputPort<Vector3D<float>>(FromHorizon::OP::OUT_VEC));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/rot_des", mux_eul_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        outer_sys->createPub("/pid_z", pid_z->getOutputPort<float>(PID_Block::OP::COMMAND));
        outer_sys->createPub( TYPE::Float3, "/vel_h_x", diff_pos->getOutputPort<Vector3D<float>>(0));

        auto mux_yaw = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_Yaw");
        outer_sys->connect( demux_ori->getOutputPort<float>(Demux3::OP::Z), mux_yaw->getInputPort<float>(Mux3::IP::X));
        outer_sys->createPub( TYPE::Float3, "/providers/yaw", mux_yaw->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));

        outer_sys->createResetTrigger("reset_controller", pid_z);
        auto update_pid_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::PID_UPDATE, "/update_controller/pid/outer");
        outer_sys->connectExternalTrigger(update_pid_trig, pid_x);
        outer_sys->connectExternalTrigger(update_pid_trig, pid_y);
        outer_sys->connectExternalTrigger(update_pid_trig, pid_z);
        
        outer_sys->start();

    }
    
} // namespace HEAR
