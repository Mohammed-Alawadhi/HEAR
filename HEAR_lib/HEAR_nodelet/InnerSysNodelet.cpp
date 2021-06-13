
#include "InnerSysNodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(HEAR::InnerSysNodelet, nodelet::Nodelet)

namespace HEAR
{
    InnerSysNodelet::~InnerSysNodelet(){
        delete inner_sys;
    }
    void InnerSysNodelet::onInit(){
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        inner_sys = new RosSystem(nh, pnh, FREQUENCY, "InnerLoop");

        // creating Blocks
        auto filt_angle_rate = inner_sys->createBlock(BLOCK_ID::BW_FILT2, "Filt_angle_rate", TYPE::Float3);
        ((BWFilter2<Vector3D<float>>*)filt_angle_rate)->setCoeff(BWFilt2_coeff::coeff_N200C50);
        auto demux_angle_rate = inner_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_AngleRate");
        auto demux_ori = inner_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_Ori");
        auto mux_rpy = inner_sys->createBlock(BLOCK_ID::MUX3, "Mux_Ori");
        auto eul2Rb_des = inner_sys->createBlock(BLOCK_ID::EUL2ROT, "Eul_to_Rb_des");
        auto eul2Rb = inner_sys->createBlock(BLOCK_ID::EUL2ROT, "Eul_to_Rb");
        auto roterr2angerr = inner_sys->createBlock(BLOCK_ID::ROTDIFF2ROD, "RotErr_to_AngErr");
        auto demux_angerr = inner_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_AngleErr");
        auto pid_roll = inner_sys->createBlock(BLOCK_ID::PID, "Pid_Roll"); ((PID_Block*)pid_roll)->setPID_ID(PID_ID::PID_ROLL);
        auto pid_pitch = inner_sys->createBlock(BLOCK_ID::PID, "Pid_Pitch"); ((PID_Block*)pid_pitch)->setPID_ID(PID_ID::PID_PITCH);
        auto pid_yaw = inner_sys->createBlock(BLOCK_ID::PID, "Pid_Yaw"); ((PID_Block*)pid_yaw)->setPID_ID(PID_ID::PID_YAW);
        auto pid_yaw_rt = inner_sys->createBlock(BLOCK_ID::PID, "Pid_Yaw_Rate"); ((PID_Block*)pid_yaw_rt)->setPID_ID(PID_ID::PID_YAW_RATE);
        auto sum_ref_yaw_rt = inner_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_Yaw_rt");
        auto mux_angle_u = inner_sys->createBlock(BLOCK_ID::MUX3, "Mux_Angle_u");
        auto sat_yaw = inner_sys->createBlock(BLOCK_ID::SATURATION, "Sat_yaw"); ((Saturation*)sat_yaw)->setClipValue(YAW_SAT_VALUE);

        //connecting blocks
        providers = new ROSUnit_PoseProvider (nh);
        auto ori_port = inner_sys->createExternalInputPort<Vector3D<float>>("Ori_port");
        auto angle_rate_port = inner_sys->createExternalInputPort<Vector3D<float>>("Angle_rt_port");
        inner_sys->connectExternalInput(ori_port, providers->registerImuOri("/filter/quaternion"));
        inner_sys->connectExternalInput(ori_port, demux_ori->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        inner_sys->connectExternalInput(angle_rate_port, providers->registerImuAngularRate("/imu/angular_velocity"));
        inner_sys->connectExternalInput(angle_rate_port, filt_angle_rate->getInputPort<Vector3D<float>>(0));
        inner_sys->connect(filt_angle_rate->getOutputPort<Vector3D<float>>(0), demux_angle_rate->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));

        inner_sys->createSub( TYPE::Float3, "/rot_des", eul2Rb_des->getInputPort<Vector3D<float>>(Eul2Rot::IP::EUL_ANGLES));
        inner_sys->createSub( "/providers/yaw", mux_rpy->getInputPort<float>(Mux3::IP::Z));
        inner_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::X), mux_rpy->getInputPort<float>(Mux3::IP::X));
        inner_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Y), mux_rpy->getInputPort<float>(Mux3::IP::Y));
        inner_sys->connect(mux_rpy->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT), eul2Rb->getInputPort<Vector3D<float>>(Eul2Rot::IP::EUL_ANGLES));

        inner_sys->createSub( TYPE::Float3,"/fi_des", roterr2angerr->getInputPort<Vector3D<float>>(FbLinearizer::RotDiff2Rod::IP::F_IDES));
        inner_sys->connect(eul2Rb_des->getOutputPort<tf2::Matrix3x3>(Eul2Rot::OP::ROT_MAT), roterr2angerr->getInputPort<tf2::Matrix3x3>(FbLinearizer::RotDiff2Rod::IP::R_BDES_I));
        inner_sys->connect(eul2Rb->getOutputPort<tf2::Matrix3x3>(Eul2Rot::OP::ROT_MAT), roterr2angerr->getInputPort<tf2::Matrix3x3>(FbLinearizer::RotDiff2Rod::IP::R_I_B));
        inner_sys->connect(roterr2angerr->getOutputPort<Vector3D<float>>(FbLinearizer::RotDiff2Rod::OP::ROD_ANGLES), demux_angerr->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));

        inner_sys->connect(demux_angerr->getOutputPort<float>(Demux3::OP::X), pid_roll->getInputPort<float>(PID_Block::IP::ERROR));
        inner_sys->connect(demux_angle_rate->getOutputPort<float>(Demux3::OP::X), pid_roll->getInputPort<float>(PID_Block::IP::PV_DOT));
        inner_sys->connect(demux_angerr->getOutputPort<float>(Demux3::OP::Y), pid_pitch->getInputPort<float>(PID_Block::IP::ERROR));
        inner_sys->connect(demux_angle_rate->getOutputPort<float>(Demux3::OP::Y), pid_pitch->getInputPort<float>(PID_Block::IP::PV_DOT));
        inner_sys->connect(demux_angerr->getOutputPort<float>(Demux3::OP::Z), pid_yaw->getInputPort<float>(PID_Block::IP::ERROR));

        inner_sys->connect(pid_yaw->getOutputPort<float>(PID_Block::OP::COMMAND), sat_yaw->getInputPort<float>(Saturation::IP::INPUT));
        inner_sys->connect(sat_yaw->getOutputPort<float>(Saturation::OP::OUTPUT), sum_ref_yaw_rt->getInputPort<float>(Sum::IP::OPERAND1));
        inner_sys->connect(demux_angle_rate->getOutputPort<float>(Demux3::OP::Z), sum_ref_yaw_rt->getInputPort<float>(Sum::IP::OPERAND2));
        inner_sys->connect(sum_ref_yaw_rt->getOutputPort<float>(Sum::OP::OUTPUT), pid_yaw_rt->getInputPort<float>(PID_Block::IP::ERROR));

        inner_sys->connect(pid_roll->getOutputPort<float>(PID_Block::OP::COMMAND), mux_angle_u->getInputPort<float>(Mux3::IP::X));
        inner_sys->connect(pid_pitch->getOutputPort<float>(PID_Block::OP::COMMAND), mux_angle_u->getInputPort<float>(Mux3::IP::Y));
        inner_sys->connect(pid_yaw_rt->getOutputPort<float>(PID_Block::OP::COMMAND), mux_angle_u->getInputPort<float>(Mux3::IP::Z));

        inner_sys->createPub(TYPE::Float3, "/angle_u", mux_angle_u->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        inner_sys->createPub( "/thrust_cmd", roterr2angerr->getOutputPort<float>(FbLinearizer::RotDiff2Rod::OP::THRUST));
        inner_sys->createPub(TYPE::Float3, "body_ori", mux_rpy->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));

        auto update_pid_trig = inner_sys->createUpdateTrigger(UPDATE_MSG_TYPE::PID_UPDATE, "update_controller/pid/inner");
        inner_sys->connectExternalTrigger(update_pid_trig, pid_roll);
        inner_sys->connectExternalTrigger(update_pid_trig, pid_pitch);
        inner_sys->connectExternalTrigger(update_pid_trig, pid_yaw);
        inner_sys->connectExternalTrigger(update_pid_trig, pid_yaw_rt);

        inner_sys->start();

    }
    
} // namespace HEAR
