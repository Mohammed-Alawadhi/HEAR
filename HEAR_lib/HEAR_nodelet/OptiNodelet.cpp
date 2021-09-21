
#include "OptiNodelet.h"
#include <pluginlib/class_list_macros.h>



PLUGINLIB_EXPORT_CLASS(HEAR::OptiNodelet, nodelet::Nodelet)

namespace HEAR
{
    OptiNodelet::~OptiNodelet(){
        delete sys;
    }
    void OptiNodelet::onInit(){
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        sys = new RosSystem(nh, pnh, FREQUENCY, "Opti Providers");

        providers = new ROSUnit_PoseProvider(nh);
        auto pos_port = sys->createExternalInputPort<Vector3D<float>>("Pos_port");
        auto ori_port = sys->createExternalInputPort<Vector3D<float>>("Ori_port");

        auto opti_port = providers->registerOptiPose("/Robot_1/pose");
        sys->connectExternalInput(pos_port, opti_port[0]);
        sys->connectExternalInput(ori_port, opti_port[1]);

        // connect publishers to KF output ports
        sys->createPub<Vector3D<float>>(TYPE::Float3, "/opti/pos", ((Block*)pos_port)->getOutputPort<Vector3D<float>>(0));
        sys->createPub<Vector3D<float>>(TYPE::Float3, "/opti/ori", ((Block*)ori_port)->getOutputPort<Vector3D<float>>(0));

        sys->start();
    }
    
} // namespace HEAR
