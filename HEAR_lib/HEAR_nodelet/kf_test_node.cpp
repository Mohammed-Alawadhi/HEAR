#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"

using namespace HEAR;

int main(int argc, char **argv){
    ros::init(argc, argv, "kf_test_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RosSystem* sys =  new RosSystem(nh, pnh, 200, "KF System");    
    ROSUnit_PoseProvider* providers = new ROSUnit_PoseProvider(nh);

    auto opti_port = providers->registerOptiPose("/Robot_1/pose");
    auto pos_port = sys->createExternalInputPort<Vector3D<float>>("Pos_opti_port");
    auto ori_port = sys->createExternalInputPort<Vector3D<float>>("Ori_opti_port");
    sys->connectExternalInput(pos_port, opti_port[0]);
    sys->connectExternalInput(ori_port, opti_port[0]);
    auto imu_port = providers->registerImuOri("/filter/quaternion");
    auto angle_rate_port = providers->registerImuAngularRate("/imu/angular_velocity");
    auto acc_port = providers->registerImuAcceleration("/imu/acceleration");
    auto imu_ori_port = sys->createExternalInputPort<Vector3D<float>>("Ori_imu_port");
    auto imu_ang_vel_port = sys->createExternalInputPort<Vector3D<float>>("Ang_vel_port");
    auto imu_acc_port = sys->createExternalInputPort<Vector3D<float>>("Acc_port");
    sys->connectExternalInput(imu_ori_port, imu_port);
    sys->connectExternalInput(imu_ang_vel_port, angle_rate_port);
    sys->connectExternalInput(imu_ori_port, acc_port);

    // connect ports to KF input ports

    
    // connect publishers to KF output ports

    sys->start();
    ros::spin();

}