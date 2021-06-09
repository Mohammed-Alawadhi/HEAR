
#include "HEAR_nodelet/OuterSysNodelet.h"
#include <pluginlib/class_list_macros.h>

#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_control/BWFilter.hpp"

PLUGINLIB_EXPORT_CLASS(HEAR::OuterSysNodelet, nodelet::Nodelet)

namespace HEAR
{
    void OuterSysNodelet::onInit(){
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        auto outer_sys = new RosSystem(nh, pnh, 120, "OuterLoop");

        // creating Blocks
        auto diff_pos = outer_sys->createBlock(BLOCK_ID::DIFFERENTIATOR, "Pos_Derivative", TYPE::Float3);
        auto pos_filt = outer_sys->createBlock(BLOCK_ID::BW_FILT2, "Vel_Filt", TYPE::Float3);
        ((BWFilter2<Vector3D<float>>*)pos_filt)->setCoeff(BWFilt2_coeff::coeff_120Hz_2nd_butter_5hz);
        auto demux_opt_ori = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Demux_Opti_Ori");
        auto to_horizon_pos = outer_sys->createBlock(BLOCK_ID::TOHORIZON, "ToHorizon_Pos");
        auto to_horizon_vel = outer_sys->createBlock(BLOCK_ID::TOHORIZON, "ToHorizon_Vel");
        auto pos_h_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Pos_H_Demux");
        auto vel_h_demux = outer_sys->createBlock(BLOCK_ID::DEMUX3, "Vel_H_Demux");
        auto sum_ref_x = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_x");
        auto sum_ref_y = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_y");
        auto sum_ref_z = outer_sys->createBlock(BLOCK_ID::SUM, "Sum_Ref_z");
        auto pid_x = outer_sys->createBlock(BLOCK_ID::PID, "Pid_x");
        auto pid_y = outer_sys->createBlock(BLOCK_ID::PID, "Pid_y");
        auto pid_z = outer_sys->createBlock(BLOCK_ID::PID, "Pid_z");
        auto sat_x = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_x");
        auto sat_y = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_y");
        auto sat_z = outer_sys->createBlock(BLOCK_ID::SATURATION, "Sat_z");

    }
    
} // namespace HEAR
