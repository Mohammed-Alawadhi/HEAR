#include "HEAR_control/PID_Block.hpp"
#include "HEAR_ROS/ROSUnit_FloatPub.hpp"
#include "HEAR_ROS/ROSUnit_FloatSub.hpp"
#include "HEAR_core/System.hpp"
#include "HEAR_core/DataTypes.hpp"

#include "ros/ros.h"

//using namespace HEAR;

const int FREQ = 200;

int main(int argc, char** argv) {
    ros::init(argc, argv, "lib_test");
    ros::NodeHandle nh;
    ros::Rate rate(FREQ);

    auto simpleSys = new HEAR::System(FREQ);

    auto Error_port = simpleSys->createExternalInputPort<float>(HEAR::TYPE::Float, "Error");
    auto PID_generic = simpleSys->addBlock(new HEAR::PID_Block(1.f/FREQ), "PID_GENERIC");
    auto command_port = simpleSys->createExternalOutputPort<float>(HEAR::TYPE::Float, "Command");

    simpleSys->connectToExternalInput<float>(Error_port, PID_generic, HEAR::PID_Block::IP::ERROR);
    simpleSys->connectToExternalOutput<float>(PID_generic, command_port, HEAR::ExternalPort::IP::INPUT);
    
    HEAR::ROSUnitFloatSub sub_float(nh);
    auto o_port = sub_float.registerSubscriber("/ref");
    simpleSys->getExternalInputPort<float>(Error_port)->connect(o_port);

    HEAR::ROSUnitFloatPub pub_float(nh);
    auto i_port = pub_float.registerPublisher("/out");

    i_port->connect(simpleSys->getExternalOutputPort<float>(command_port));

    simpleSys->init();
    simpleSys->printSystem();
    simpleSys->execute();

    while (ros::ok())
    {
        pub_float.process();
        ros::spinOnce();
        rate.sleep();
        /* code */
    }
    simpleSys->terminate();

}
