#pragma once

#include <HEAR_core/Block.hpp>
#include "HEAR_core/ExternalPort.hpp"
#include "ros/ros.h"
#include "HEAR_core/DataTypes.hpp"
#include "std_msgs/Float32.h"

namespace HEAR{
class ROSUnitFloatSub : public Block{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    static const int capacity = 10; 
    static ExternalOutputPort<float>* ports[capacity];
public:
    ROSUnitFloatSub (const ros::NodeHandle& nh);
    static int internal_counter;
    ExternalOutputPort<float>* registerSubscriber(const std::string& );
    static void(*callbackFunctionPointer[capacity])(const std_msgs::Float32::ConstPtr&);
    static void callback0(const std_msgs::Float32::ConstPtr&);
    static void callback1(const std_msgs::Float32::ConstPtr&);
    static void callback2(const std_msgs::Float32::ConstPtr&);
    static void callback3(const std_msgs::Float32::ConstPtr&);
    static void callback4(const std_msgs::Float32::ConstPtr&);
    static void callback5(const std_msgs::Float32::ConstPtr&);
    static void callback6(const std_msgs::Float32::ConstPtr&);
    static void callback7(const std_msgs::Float32::ConstPtr&);
    static void callback8(const std_msgs::Float32::ConstPtr&);
    static void callback9(const std_msgs::Float32::ConstPtr&);

};

}