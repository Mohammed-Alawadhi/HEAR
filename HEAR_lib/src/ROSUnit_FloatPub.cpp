
#include "HEAR_ROS/ROSUnit_FloatPub.hpp"

namespace HEAR{

ROSUnitFloatPub::ROSUnitFloatPub(ros::NodeHandle& nh) : nh_(nh), Block(BLOCK_ID::ROSFLOATPUB){
   
}

ExternalInputPort<float>* ROSUnitFloatPub::registerPublisher(const std::string &topic_name){
    port = new ExternalInputPort<float>(BLOCK_ID::EXT_IP);
    pub_ = nh_.advertise<std_msgs::Float32>(topic_name, 1, true);
    return port;
}
void ROSUnitFloatPub::process(){
    if (port != NULL){
        std_msgs::Float32 msg;
        port->read(msg.data);
        pub_.publish(msg);
    }
}

}