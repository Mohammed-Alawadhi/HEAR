#ifndef ROSUNITFLOATSUB_HPP
#define ROSUNITFLOATSUB_HPP

#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "ros/ros.h"
#include "HEAR_core/DataTypes.hpp"
#include "std_msgs/Float32.h"

namespace HEAR{
class ROSUnitFloatSub : public Block{
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub;
    ExternalOutputPort<float>* port;
    void callback(const std_msgs::Float32::ConstPtr&);
public:
    ROSUnitFloatSub (const ros::NodeHandle& nh, int b_uid);
    ExternalOutputPort<float>* registerSubscriber(const std::string& );
    void process(){}
};

}
#endif