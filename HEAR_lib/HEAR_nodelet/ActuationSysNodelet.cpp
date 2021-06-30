
#include "ActuationSysNodelet.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(HEAR::ActuationSysNodelet, nodelet::Nodelet)

namespace HEAR
{
    ActuationSysNodelet::~ActuationSysNodelet(){
        delete actuation_sys;
    }
    void ActuationSysNodelet::onInit(){
        ros::NodeHandle nh(getNodeHandle());
        ros::NodeHandle pnh(getPrivateNodeHandle());
        actuation_sys = new RosSystem(nh, pnh, FREQUENCY, "ActuationLoop");

        // creating Blocks
        auto hexa = actuation_sys->createBlock(BLOCK_ID::HEXAACTUATIONSYSTEM, "Hexa");
        ((HexaActuationSystem*)hexa)->init(FREQUENCY);
        ((HexaActuationSystem*)hexa)->setHbTol(250);
        #ifdef BIG_HEXA
        ((HexaActuationSystem*)hexa)->setESCValues(1165 ,1000, 2000);
        #else
        ((HexaActuationSystem*)hexa)->setESCValues(1140 ,1000, 2000);
        #endif

        actuation_sys->createSub(TYPE::Float3, "/angle_u", hexa->getInputPort<Vector3D<float>>(HexaActuationSystem::IP::BODY_RATE_CMD));
        actuation_sys->createSub("/thrust_cmd", hexa->getInputPort<float>(HexaActuationSystem::IP::THRUST_CMD));

        actuation_sys->createPub(TYPE::FloatVec, "/actuation_cmd", hexa->getOutputPort<std::vector<float>>(HexaActuationSystem::OP::MOTOR_CMD));

        actuation_sys->createUpdateTrigger(UPDATE_MSG_TYPE::BOOL_MSG, "/arm", hexa);
        _hb_sub = nh.subscribe("/heartbeat", 10, &HexaActuationSystem::heartbeatCb, (HexaActuationSystem*)hexa);
        
        actuation_sys->start();

        std::cout << "Created all blocks\n";

    }
    
} // namespace HEAR
