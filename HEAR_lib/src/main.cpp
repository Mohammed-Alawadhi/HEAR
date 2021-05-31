

#include "HEAR_control/PID_Block.hpp"
#include "HEAR_ROS/ROSUnit_FloatPub.hpp"
#include "HEAR_ROS/ROSUnit_FloatSub.hpp"
#include "HEAR_core/System.hpp"
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/ExternalTrigger.hpp"
#include "HEAR_ROS/ROSUnit_ResetSrv.hpp"
#include "HEAR_control/BWFilter.hpp"
#include "Vector3D.hpp"
#include "ros/ros.h"
#include <iostream>

using namespace HEAR;

const int FREQ = 200;

int main(int argc, char** argv) {
    ros::init(argc, argv, "tesing_node");
    ros::NodeHandle nh;
    ros::Rate rate(FREQ);

    auto simpleSys = new System(FREQ);
    std::cout<< "creating blocks\n";


    auto Error_port = simpleSys->createExternalInputPort<float>(TYPE::Float, "Error");
    std::cout<< "creating ext input\n";

    auto pid_Block = new PID_Block();
    std::cout<< "created pid block\n";

    auto PID_generic = simpleSys->addBlock(pid_Block, "PID_GENERIC");
    std::cout<< "added pid block\n";

    auto command_port = simpleSys->createExternalOutputPort<float>(TYPE::Float, "Command");

    simpleSys->connectToExternalInput<float>(Error_port, PID_generic, PID_Block::IP::ERROR);
    std::cout<< "connected ext input\n";

    simpleSys->connectToExternalOutput<float>(PID_generic, PID_Block::OP::OUTPUT, command_port);
    std::cout<< "connected ext output\n";

    ROSUnitFloatSub sub_float(nh);
    std::cout<< "registering subscriber\n";
    auto o_port = sub_float.registerSubscriber("/ref");
    simpleSys->getExternalInputPort<float>(Error_port)->connect(o_port);

    ROSUnitFloatPub pub_float(nh);
    auto i_port = pub_float.registerPublisher("/out");

    i_port->connect(simpleSys->getExternalOutputPort<float>(command_port));

    auto bwFilt = new BWFilter2<Vector3D<float>>(BWFilt2_coeff::coeff_N200C50, TYPE::Float3);

    /// adding external trigger    
    // auto ros_reset_trig = new ROSUnit_ResetServer(nh);
    // auto reset_trig = simpleSys->addExternalTrigger(ros_reset_trig->registerServer("reset_z"), "Reset Z");
    // simpleSys->connectExtTrig(PID_generic, reset_trig);
//    resetTrig->resetCallback


    std::cout<< "system created\n";
    simpleSys->init();
    std::cout<< "system initialized\n";
    simpleSys->printSystem();
    std::cout<< "starting system\n";
    
    simpleSys->execute();
    std::cout<< "system started\n";


    while(ros::ok()){
        ros::spinOnce();
        pub_float.process();
        rate.sleep();
    }
    simpleSys->terminate();

}
