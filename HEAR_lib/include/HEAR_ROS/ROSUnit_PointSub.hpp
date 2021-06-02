#ifndef ROSUNIT_POINTSUB_HPP
#define ROSUNIT_POINTSUB_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Vector3D.hpp"

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

namespace HEAR{
class ROSUnitPointSub : public Block{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    static const int capacity = 10; 
    static int internal_counter;
    static ExternalOutputPort<Vector3D<float>>* ports[capacity];
    static void(*callbackFunctionPointer[capacity])(const geometry_msgs::Point::ConstPtr&);
    static void callback0(const geometry_msgs::Point::ConstPtr&);
    static void callback1(const geometry_msgs::Point::ConstPtr&);
    static void callback2(const geometry_msgs::Point::ConstPtr&);
    static void callback3(const geometry_msgs::Point::ConstPtr&);
    static void callback4(const geometry_msgs::Point::ConstPtr&);
    static void callback5(const geometry_msgs::Point::ConstPtr&);
    static void callback6(const geometry_msgs::Point::ConstPtr&);
    static void callback7(const geometry_msgs::Point::ConstPtr&);
    static void callback8(const geometry_msgs::Point::ConstPtr&);
    static void callback9(const geometry_msgs::Point::ConstPtr&);

public:
    ROSUnitPointSub (const ros::NodeHandle& nh);
    ExternalOutputPort<Vector3D<float>>* registerSubscriber(const std::string& );
    void process(){}
};

}

#endif