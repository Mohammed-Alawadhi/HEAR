
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

        //////////// creating blocks /////////////
        auto diff_pos = outer_sys->createBlock(BLOCK_ID::DIFFERENTIATOR, "Pos_Derivative", TYPE::Float3); ((Differentiator<Vector3D<float>>*)diff_pos)->supPeak(0.3);
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
        auto sum_ref_vel_x = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_Vel_x");
        auto sum_ref_vel_y = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_Vel_y");
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
        auto sum_bias_z = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_bias_z"); ((Sum*)sum_bias_z)->setOperation(Sum::OPERATION::ADD);

        // feedforward acceleration blocks
        auto sum_acc_x = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_acc_x"); ((Sum*)sum_acc_x)->setOperation(Sum::OPERATION::ADD);
        auto sum_acc_y = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_acc_y"); ((Sum*)sum_acc_y)->setOperation(Sum::OPERATION::ADD);
        auto grav_scale = outer_sys->createBlock(BLOCK_ID::GAIN, "Grav_Normalize"); ((Gain*)grav_scale)->setGain(1.0/9.8);
        auto hold_thrust_val = outer_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Thrust_Val");
        auto acc_ref_gain_x = outer_sys->createBlock(BLOCK_ID::MULTIPLY, "Acc_Ref_Gain_x");
        auto acc_ref_gain_y = outer_sys->createBlock(BLOCK_ID::MULTIPLY, "Acc_Ref_Gain_y");

        //////// creating MRFT specific blocks ////////////
        auto mrft_x = outer_sys->createBlock(BLOCK_ID::MRFT, "Mrft_x"); ((MRFT_Block*)mrft_x)->setMRFT_ID(MRFT_ID::MRFT_X);
        auto mrft_y = outer_sys->createBlock(BLOCK_ID::MRFT, "Mrft_y"); ((MRFT_Block*)mrft_y)->setMRFT_ID(MRFT_ID::MRFT_Y);
        auto mrft_z = outer_sys->createBlock(BLOCK_ID::MRFT, "Mrft_z"); ((MRFT_Block*)mrft_z)->setMRFT_ID(MRFT_ID::MRFT_Z);
        auto mrft_sw_x = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Mrft_Sw_x");
        auto mrft_sw_y = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Mrft_Sw_y");
        auto mrft_sw_z = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Mrft_Sw_z");
        auto med_filt_x = outer_sys->createBlock(BLOCK_ID::MEDIAN_FILTER, "Med_Filt_x"); ((MedianFilter*)med_filt_x)->setWinSize(50);
        auto med_filt_y = outer_sys->createBlock(BLOCK_ID::MEDIAN_FILTER, "Med_Filt_y"); ((MedianFilter*)med_filt_y)->setWinSize(50);
        auto med_filt_z = outer_sys->createBlock(BLOCK_ID::MEDIAN_FILTER, "Med_Filt_z"); ((MedianFilter*)med_filt_z)->setWinSize(50);
        auto hold_ref_x = outer_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Ref_x");
        auto hold_ref_y = outer_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Ref_y");
        auto hold_ref_z = outer_sys->createBlock(BLOCK_ID::HOLDVAL, "Hold_Ref_z");

        //////// creating SLAM specific Blocks ///////////
        auto diff_slam_pos = outer_sys->createBlock(BLOCK_ID::DIFFERENTIATOR, "Pos_Derivative", TYPE::Float3); ((Differentiator<Vector3D<float>>*)diff_slam_pos)->supPeak(0.3);
        auto pos_slam_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Pos_SLAM_Demux");
        auto pos_sw = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH3, "Pos_sw");
        auto vel_sw = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH3, "Vel_sw");
        auto ori_sw = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH3, "Ori_sw");
        auto ref_sw_x = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Ref_Sw_x");
        auto ref_sw_y = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Ref_Sw_y");
        auto ref_sw_z = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Ref_Sw_z");
        auto prov_sw_x = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Prov_Sw_x");
        auto prov_sw_y = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Prov_Sw_y");
        auto prov_sw_z = outer_sys->createBlock(BLOCK_ID::INVERTED_SWITCH, "Prov_Sw_z");
                
        ////////////// connecting blocks /////////

        // external input configuration 
        providers = new ROSUnit_PoseProvider(nh);
        auto pos_port = outer_sys->createExternalInputPort<Vector3D<float>>("Pos_Port");
        auto ori_port = outer_sys->createExternalInputPort<Vector3D<float>>("Ori_Port");
        auto opti_port = providers->registerOptiPose("/Robot_1/pose");
        outer_sys->connectExternalInput(pos_port, opti_port[0]);
        outer_sys->connectExternalInput(ori_port, opti_port[1]);

        // external input for slam
        providers_slam = new ROSUnit_SLAM(nh);
        providers_slam->connectInputs(((Block*)pos_port)->getOutputPort<Vector3D<float>>(0), ((Block*)ori_port)->getOutputPort<Vector3D<float>>(0));
        auto pos_slam_port = outer_sys->createExternalInputPort<Vector3D<float>>("Pos_SLAM_Port");
        auto ori_slam_port = outer_sys->createExternalInputPort<Vector3D<float>>("Ori_SLAM_Port");
        auto slam_port = providers_slam->registerSLAM("/zedm/zed_node/odom");
        outer_sys->connectExternalInput(pos_slam_port, slam_port[0]);
        outer_sys->connectExternalInput(ori_slam_port, slam_port[1]);
        outer_sys->connectExternalInput(pos_slam_port, pos_slam_demux->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        outer_sys->connectExternalInput(pos_slam_port, diff_slam_pos->getInputPort<Vector3D<float>>(0));
        outer_sys->connectExternalInput(pos_slam_port, pos_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NO));
        outer_sys->connectExternalInput(ori_slam_port, ori_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NO));
        outer_sys->connect(diff_slam_pos->getOutputPort<Vector3D<float>>(0), vel_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NO));

        // connecting input data preparation blocks
        outer_sys->connectExternalInput(ori_port, ori_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NC));
        outer_sys->connect(ori_sw->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), demux_ori->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        outer_sys->connectExternalInput(pos_port, diff_pos->getInputPort<Vector3D<float>>(0));
        outer_sys->connect(diff_pos->getOutputPort<Vector3D<float>>(0), pos_filt->getInputPort<Vector3D<float>>(0));
        outer_sys->connectExternalInput(pos_port, pos_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NC));
        outer_sys->connect(pos_sw->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), to_horizon_pos->getInputPort<Vector3D<float>>(ToHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), to_horizon_pos->getInputPort<float>(ToHorizon::IP::YAW));
        outer_sys->connect(pos_filt->getOutputPort<Vector3D<float>>(0), vel_sw->getInputPort<Vector3D<float>>(InvertedSwitch3::IP::NC));
        outer_sys->connect(vel_sw->getOutputPort<Vector3D<float>>(InvertedSwitch3::OP::COM), to_horizon_vel->getInputPort<Vector3D<float>>(ToHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), to_horizon_vel->getInputPort<float>(ToHorizon::IP::YAW));
        outer_sys->connect(to_horizon_pos->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC), pos_h_demux->getInputPort<Vector3D<float>>(Demux3::INPUT));
        outer_sys->connect(to_horizon_vel->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC), vel_h_demux->getInputPort<Vector3D<float>>(Demux3::INPUT));

        // feedforward acceleration thrust scaling
        outer_sys->createSub("/thrust_cmd", hold_thrust_val->getInputPort<float>(HoldVal::IP::INPUT));
        outer_sys->connect(hold_thrust_val->getOutputPort<float>(HoldVal::OP::OUTPUT), grav_scale->getInputPort<float>(Gain::IP::INPUT));
        outer_sys->connect(grav_scale->getOutputPort<float>(Gain::OP::OUTPUT), acc_ref_gain_x->getInputPort<float>(Multiply::IP::INPUT_0));
        outer_sys->connect(grav_scale->getOutputPort<float>(Gain::OP::OUTPUT), acc_ref_gain_y->getInputPort<float>(Multiply::IP::INPUT_0));
        
        // connecting x control sys blocks
        outer_sys->createSub("/waypoint_reference/x", ref_sw_x->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pos_slam_demux->getOutputPort<float>(Demux3::OP::X), hold_ref_x->getInputPort<float>(HoldVal::IP::INPUT));
        outer_sys->connect(hold_ref_x->getOutputPort<float>(HoldVal::OP::OUTPUT), ref_sw_x->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(ref_sw_x->getOutputPort<float>(InvertedSwitch::OP::COM), sum_ref_x->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::X), prov_sw_x->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pos_slam_demux->getOutputPort<float>(Demux3::OP::X), prov_sw_x->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(prov_sw_x->getOutputPort<float>(InvertedSwitch::OP::COM), sum_ref_x->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_ref_x->getOutputPort<float>(Sum::OP::OUTPUT), pid_x->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::X), sum_ref_vel_x->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->createSub("/waypoint_reference/vel/x", sum_ref_vel_x->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_ref_vel_x->getOutputPort<float>(Sum::OP::OUTPUT), pid_x->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->createSub("/waypoint_reference/acc/x", acc_ref_gain_x->getInputPort<float>(Multiply::IP::INPUT_1));
        outer_sys->connect(acc_ref_gain_x->getOutputPort<float>(Multiply::OP::OUTPUT), sum_acc_x->getInputPort<float>(Sum::OPERAND1));
        outer_sys->connect(pid_x->getOutputPort<float>(PID_Block::OP::COMMAND), sum_acc_x->getInputPort<float>(Sum::OPERAND2));
        outer_sys->connect(sum_acc_x->getOutputPort<float>(Sum::OP::OUTPUT), mrft_sw_x->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pid_x->getOutputPort<float>(PID_Block::OP::COMMAND), med_filt_x->getInputPort<float>(MedianFilter::IP::INPUT));
        outer_sys->connect(med_filt_x->getOutputPort<float>(MedianFilter::OP::OUTPUT), mrft_x->getInputPort<float>(MRFT_Block::IP::BIAS));
        outer_sys->connect(sum_ref_x->getOutputPort<float>(Sum::OP::OUTPUT), mrft_x->getInputPort<float>(MRFT_Block::IP::INPUT));
        outer_sys->connect(mrft_x->getOutputPort<float>(MRFT_Block::OP::COMMAND), mrft_sw_x->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(mrft_sw_x->getOutputPort<float>(InvertedSwitch::OP::COM), mux_fh_des->getInputPort<float>(Mux3::IP::X));

        // connecting y control sys blocks
        outer_sys->createSub("/waypoint_reference/y", ref_sw_y->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pos_slam_demux->getOutputPort<float>(Demux3::OP::Y), hold_ref_y->getInputPort<float>(HoldVal::IP::INPUT));
        outer_sys->connect(hold_ref_y->getOutputPort<float>(HoldVal::OP::OUTPUT), ref_sw_y->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(ref_sw_y->getOutputPort<float>(InvertedSwitch::OP::COM), sum_ref_y->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::Y), prov_sw_y->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pos_slam_demux->getOutputPort<float>(Demux3::OP::Y), prov_sw_y->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(prov_sw_y->getOutputPort<float>(InvertedSwitch::OP::COM), sum_ref_y->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_ref_y->getOutputPort<float>(Sum::OP::OUTPUT), pid_y->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::Y), sum_ref_vel_y->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->createSub("/waypoint_reference/vel/y", sum_ref_vel_y->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_ref_vel_y->getOutputPort<float>(Sum::OP::OUTPUT), pid_y->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->createSub("/waypoint_reference/acc/y", acc_ref_gain_y->getInputPort<float>(Multiply::IP::INPUT_1));
        outer_sys->connect(acc_ref_gain_y->getOutputPort<float>(Multiply::OP::OUTPUT), sum_acc_y->getInputPort<float>(Sum::OPERAND1));
        outer_sys->connect(pid_y->getOutputPort<float>(PID_Block::OP::COMMAND), sum_acc_y->getInputPort<float>(Sum::OPERAND2));
        outer_sys->connect(sum_acc_y->getOutputPort<float>(Sum::OP::OUTPUT), mrft_sw_y->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pid_y->getOutputPort<float>(PID_Block::OP::COMMAND), med_filt_y->getInputPort<float>(MedianFilter::IP::INPUT));
        outer_sys->connect(med_filt_y->getOutputPort<float>(MedianFilter::OP::OUTPUT), mrft_y->getInputPort<float>(MRFT_Block::IP::BIAS));
        outer_sys->connect(sum_ref_y->getOutputPort<float>(Sum::OP::OUTPUT), mrft_y->getInputPort<float>(MRFT_Block::IP::INPUT));
        outer_sys->connect(mrft_y->getOutputPort<float>(MRFT_Block::OP::COMMAND), mrft_sw_y->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(mrft_sw_y->getOutputPort<float>(InvertedSwitch::OP::COM), mux_fh_des->getInputPort<float>(Mux3::IP::Y));

        // connecting z control sys blocks
        outer_sys->createSub("/waypoint_reference/z", ref_sw_z->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pos_slam_demux->getOutputPort<float>(Demux3::OP::Z), hold_ref_z->getInputPort<float>(HoldVal::IP::INPUT));
        outer_sys->connect(hold_ref_z->getOutputPort<float>(HoldVal::OP::OUTPUT), ref_sw_z->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(ref_sw_z->getOutputPort<float>(InvertedSwitch::OP::COM), sum_ref_z->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(pos_h_demux->getOutputPort<float>(Demux3::OP::Z), prov_sw_z->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(pos_slam_demux->getOutputPort<float>(Demux3::OP::Z), prov_sw_z->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(prov_sw_z->getOutputPort<float>(InvertedSwitch::OP::COM), sum_ref_z->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_ref_z->getOutputPort<float>(Sum::OP::OUTPUT), pid_z->getInputPort<float>(PID_Block::IP::ERROR));
        outer_sys->connect(vel_h_demux->getOutputPort<float>(Demux3::OP::Z), pid_z->getInputPort<float>(PID_Block::IP::PV_DOT));
        outer_sys->connect(pid_z->getOutputPort<float>(PID_Block::OP::COMMAND), sum_bias_z->getInputPort<float>(Sum::IP::OPERAND1));
        outer_sys->connect(bias_z->getOutputPort<float>(Constant<float>::OP::OUTPUT), sum_bias_z->getInputPort<float>(Sum::IP::OPERAND2));
        outer_sys->connect(sum_bias_z->getOutputPort<float>(Sum::OP::OUTPUT), sat_z->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(sat_z->getOutputPort<float>(Saturation::OP::OUTPUT), mrft_sw_z->getInputPort<float>(InvertedSwitch::IP::NC));
        outer_sys->connect(sat_z->getOutputPort<float>(Saturation::OP::OUTPUT), med_filt_z->getInputPort<float>(MedianFilter::IP::INPUT));
        outer_sys->connect(med_filt_z->getOutputPort<float>(MedianFilter::OP::OUTPUT), mrft_z->getInputPort<float>(MRFT_Block::IP::BIAS));
        outer_sys->connect(sum_ref_z->getOutputPort<float>(Sum::OP::OUTPUT), mrft_z->getInputPort<float>(MRFT_Block::IP::INPUT));
        outer_sys->connect(mrft_z->getOutputPort<float>(MRFT_Block::OP::COMMAND), mrft_sw_z->getInputPort<float>(InvertedSwitch::IP::NO));
        outer_sys->connect(mrft_sw_z->getOutputPort<float>(InvertedSwitch::OP::COM), mux_fh_des->getInputPort<float>(Mux3::IP::Z));

        // connecting feedback linearization blocks
        outer_sys->connect(mux_fh_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT), fh2fi->getInputPort<Vector3D<float>>(FromHorizon::IP::INP_VEC));
        outer_sys->connect(demux_ori->getOutputPort<float>(Demux3::OP::Z), fh2fi->getInputPort<float>(FromHorizon::IP::YAW));
        outer_sys->connect(fh2fi->getOutputPort<Vector3D<float>>(FromHorizon::OP::OUT_VEC), f2rot->getInputPort<Vector3D<float>>(FbLinearizer::Force2Rot::IP::FORCE_I_DES));
        outer_sys->createSub("/waypoint_reference/yaw", f2rot->getInputPort<float>(FbLinearizer::Force2Rot::IP::YAW_REF));
        outer_sys->connect(f2rot->getOutputPort<tf2::Matrix3x3>(FbLinearizer::Force2Rot::OP::ROT_DES), rot2eul->getInputPort<tf2::Matrix3x3>(Rot2Eul::IP::ROT_MAT));

        // connecting ref angles data preparation blocks
        outer_sys->connect(rot2eul->getOutputPort<Vector3D<float>>(Rot2Eul::OP::EUL_ANGLES), demux_eul_des->getInputPort<Vector3D<float>>(Demux3::IP::INPUT));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::X), sat_x->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::Y), sat_y->getInputPort<float>(Saturation::IP::INPUT));
        outer_sys->connect(sat_x->getOutputPort<float>(Saturation::OP::OUTPUT), mux_eul_des->getInputPort<float>(Mux3::IP::X));
        outer_sys->connect(sat_y->getOutputPort<float>(Saturation::OP::OUTPUT), mux_eul_des->getInputPort<float>(Mux3::IP::Y));
        outer_sys->connect(demux_eul_des->getOutputPort<float>(Demux3::OP::Z), mux_eul_des->getInputPort<float>(Mux3::IP::Z));

        ///////////////// configuring publishers //////////////////
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/pos_horizon", to_horizon_pos->getOutputPort<Vector3D<float>>(ToHorizon::OUT_VEC));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/fh_des", mux_fh_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/fi_des", fh2fi->getOutputPort<Vector3D<float>>(FromHorizon::OP::OUT_VEC));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "/rot_des", mux_eul_des->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));
        outer_sys->createPub("/pid_z", pid_z->getOutputPort<float>(PID_Block::OP::COMMAND));
        outer_sys->createPub( TYPE::Float3, "/vel_h_x", diff_pos->getOutputPort<Vector3D<float>>(0));
        outer_sys->createPub( TYPE::Float3, "/vel_h_filt", pos_filt->getOutputPort<Vector3D<float>>(0));

        // setting publishers for opti and slam pose data
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "opti_pos", ((Block*)pos_port)->getOutputPort<Vector3D<float>>(0));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "opti_ori", ((Block*)ori_port)->getOutputPort<Vector3D<float>>(0));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "slam_pos", ((Block*)pos_slam_port)->getOutputPort<Vector3D<float>>(0));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "slam_vel", diff_slam_pos->getOutputPort<Vector3D<float>>(0));
        outer_sys->createPub<Vector3D<float>>(TYPE::Float3, "slam_ori", ((Block*)ori_slam_port)->getOutputPort<Vector3D<float>>(0));

        // configuring yaw provider for mission scenario
        auto mux_yaw = outer_sys->createBlock(BLOCK_ID::MUX3, "Mux_Yaw");
        outer_sys->connect( demux_ori->getOutputPort<float>(Demux3::OP::Z), mux_yaw->getInputPort<float>(Mux3::IP::X));
        outer_sys->createPub( TYPE::Float3, "/providers/yaw", mux_yaw->getOutputPort<Vector3D<float>>(Mux3::OP::OUTPUT));

        /////////////////// Setting External Triggers /////////////////

        //setting filter trigger
        auto enable_bwfilt_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/enable_outer_filter");
        outer_sys->connectExternalTrigger(enable_bwfilt_trig, pos_filt);

        // setting pid controller triggers
        auto rest_all_trig = outer_sys->createResetTrigger("reset_outer_controller");
        outer_sys->connectExternalTrigger(rest_all_trig, pid_x);
        outer_sys->connectExternalTrigger(rest_all_trig, pid_y);
        outer_sys->connectExternalTrigger(rest_all_trig, pid_z);
        outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/pid_z_trig", pid_z);
        auto update_pid_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::PID_UPDATE, "/update_controller/pid/outer");
        outer_sys->connectExternalTrigger(update_pid_trig, pid_x);
        outer_sys->connectExternalTrigger(update_pid_trig, pid_y);
        outer_sys->connectExternalTrigger(update_pid_trig, pid_z);

        // setting hold thrust val trigger
        outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/record_hover_thrust", hold_thrust_val);

        // MRFT X triggering configuration
        trig_srv_x = new ROSUnit_MRFTSwitchSrv(nh, "mrft_switch_x");
        auto pid_x_trig = trig_srv_x->getPIDTrig(); outer_sys->addExternalTrigger(pid_x_trig, "Pid_Trig_x");
        auto mrft_x_trig = trig_srv_x->getMRFTTrig(); outer_sys->addExternalTrigger(mrft_x_trig, "Mrft_Trig_x");
        auto mrft_sw_x_trig = trig_srv_x->registerSwitchTrig(); outer_sys->addExternalTrigger(mrft_sw_x_trig, "Mrft_Sw_Trig_x");
        outer_sys->connectExternalTrigger(pid_x_trig, pid_x);
        outer_sys->connectExternalTrigger(mrft_x_trig, mrft_x);
        outer_sys->connectExternalTrigger(mrft_x_trig, hold_ref_x);
        outer_sys->connectExternalTrigger(mrft_sw_x_trig, mrft_sw_x);
        // outer_sys->connectExternalTrigger(mrft_sw_x_trig, ref_sw_x);

        // MRFT Y triggering configuration
        trig_srv_y = new ROSUnit_MRFTSwitchSrv(nh, "mrft_switch_y");
        auto pid_y_trig = trig_srv_y->getPIDTrig(); outer_sys->addExternalTrigger(pid_y_trig, "Pid_Trig_y");
        auto mrft_y_trig = trig_srv_y->getMRFTTrig(); outer_sys->addExternalTrigger(mrft_y_trig, "Mrft_Trig_y");
        auto mrft_sw_y_trig = trig_srv_y->registerSwitchTrig(); outer_sys->addExternalTrigger(mrft_sw_y_trig, "Mrft_Sw_Trig_y");
        outer_sys->connectExternalTrigger(pid_y_trig, pid_y);
        outer_sys->connectExternalTrigger(mrft_y_trig, mrft_y);
        outer_sys->connectExternalTrigger(mrft_y_trig, hold_ref_y);
        outer_sys->connectExternalTrigger(mrft_sw_y_trig, mrft_sw_y);
        // outer_sys->connectExternalTrigger(mrft_sw_y_trig, ref_sw_y);

        // MRFT Z triggering configuration
        trig_srv_z = new ROSUnit_MRFTSwitchSrv(nh, "mrft_switch_z");
        auto pid_z_trig = trig_srv_z->getPIDTrig(); outer_sys->addExternalTrigger(pid_z_trig, "Pid_Trig_z");
        auto mrft_z_trig = trig_srv_z->getMRFTTrig(); outer_sys->addExternalTrigger(mrft_z_trig, "Mrft_Trig_z");
        auto mrft_sw_z_trig = trig_srv_z->registerSwitchTrig(); outer_sys->addExternalTrigger(mrft_sw_z_trig, "Mrft_Sw_Trig_z");
        outer_sys->connectExternalTrigger(pid_z_trig, pid_z);
        outer_sys->connectExternalTrigger(mrft_z_trig, mrft_z);
        outer_sys->connectExternalTrigger(mrft_z_trig, hold_ref_z);
        outer_sys->connectExternalTrigger(mrft_sw_z_trig, mrft_sw_z);
        // outer_sys->connectExternalTrigger(mrft_sw_z_trig, ref_sw_z);

        // setting mrft controllers triggers        
        auto mrft_update_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::MRFT_UPDATE, "/update_controller/mrft") ;
        outer_sys->connectExternalTrigger(mrft_update_trig, mrft_x);
        outer_sys->connectExternalTrigger(mrft_update_trig, mrft_y);
        outer_sys->connectExternalTrigger(mrft_update_trig, mrft_z);

        // setting slam provider switches
        auto slam_sw_trig = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/slam_switch");
        outer_sys->connectExternalTrigger(slam_sw_trig, pos_sw);
        outer_sys->connectExternalTrigger(slam_sw_trig, vel_sw);
        outer_sys->connectExternalTrigger(slam_sw_trig, ori_sw);
        // outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/slam_switch_x", prov_sw_x);  //// NEED to add prov for velocity also
        // outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/slam_switch_y", prov_sw_y);
        // outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/slam_switch_z", prov_sw_z);

        auto mrft_slam_trig_x = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/slam_mrft_switch_x", ref_sw_x);
        auto mrft_slam_trig_y = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/slam_mrft_switch_y", ref_sw_y);
        auto mrft_slam_trig_z = outer_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/slam_mrft_switch_z", ref_sw_z);
        outer_sys->connectExternalTrigger(mrft_slam_trig_x, prov_sw_x);
        outer_sys->connectExternalTrigger(mrft_slam_trig_y, prov_sw_y);
        outer_sys->connectExternalTrigger(mrft_slam_trig_z, prov_sw_z);

        /////////////// initializing and starting the outer loop system  /////////////////
        outer_sys->start();

    }
    
} // namespace HEAR
