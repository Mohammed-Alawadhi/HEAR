#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"
#include "HEAR_ROS/ROSUnit_SLAM.hpp"

#define use_SLAM

using namespace HEAR;

int main(int argc, char **argv){
    ros::init(argc, argv, "kf_test_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RosSystem* sys =  new RosSystem(nh, pnh, 200, "KF System");    
    ROSUnit_PoseProvider* providers = new ROSUnit_PoseProvider(nh);
    auto angle_rate_port = providers->registerImuAngularRate("/imu/angular_velocity");
    auto acc_port = providers->registerImuAcceleration("/imu/acceleration");
    auto imu_ang_vel_port = sys->createExternalInputPort<Vector3D<float>>("Ang_vel_port");
    auto imu_acc_port = sys->createExternalInputPort<Vector3D<float>>("Acc_port");

    sys->connectExternalInput(imu_ang_vel_port, angle_rate_port);
    sys->connectExternalInput(imu_acc_port, acc_port);

    auto kf = sys->createBlock(BLOCK_ID::KF, "KF_3D");
    sys->connectExternalInput(imu_ang_vel_port, kf->getInputPort<Vector3D<float>>(KF3D::IP::GYRO));
    sys->connectExternalInput(imu_acc_port, kf->getInputPort<Vector3D<float>>(KF3D::IP::ACC));

    #ifdef use_SLAM
    sys->createSub(TYPE::Float3, "/slam/pos", kf->getInputPort<Vector3D<float>>(KF3D::IP::POS));
    sys->createSub(TYPE::Float3, "/slam/ori", kf->getInputPort<Vector3D<float>>(KF3D::IP::ANGLES));
    #else
    sys->createSub(TYPE::Float3, "/opti/pos", kf->getInputPort<Vector3D<float>>(KF3D::IP::POS));
    sys->createSub(TYPE::Float3, "/opti/ori", kf->getInputPort<Vector3D<float>>(KF3D::IP::ANGLES));
    #endif
    
    // connect publishers to KF output ports
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/kf/position", kf->getOutputPort<Vector3D<float>>(KF3D::OP::PRED_POS));
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/kf/velocity", kf->getOutputPort<Vector3D<float>>(KF3D::OP::PRED_VEL));
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/kf/angles", kf->getOutputPort<Vector3D<float>>(KF3D::OP::PRED_ANG));
    sys->createPub<Vector3D<float>>(TYPE::Float3, "/kf/acc_bias", kf->getOutputPort<Vector3D<float>>(KF3D::OP::PRED_ACC_B));

    sys->start();
    ros::spin();

}