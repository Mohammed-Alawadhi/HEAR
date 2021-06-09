#include "HEAR_ROS/ROSUnit_FloatSub.hpp"

namespace HEAR{



ROSUnitFloatSub::ROSUnitFloatSub(const ros::NodeHandle& nh, int b_uid): nh_(nh), Block(BLOCK_ID::ROSFLOATSUB, b_uid){

}

ExternalOutputPort<float>* ROSUnitFloatSub::registerSubscriber(const std::string& topic_name){
        this->sub = nh_.subscribe<std_msgs::Float32>(topic_name, 10, &ROSUnitFloatSub::callback, this);
}

void ROSUnitFloatSub::callback(const std_msgs::Float32::ConstPtr& msg){
    port->write(msg->data);
}

}