
#include "HEAR_ROS/ROSUnit_FloatArrPub.hpp"

namespace HEAR{

ROSUnitFloatArrPub::ROSUnitFloatArrPub(ros::NodeHandle& nh) : nh_(nh), Block(BLOCK_ID::ROSFLOATARRPUB){
   
}

ExternalInputPort<std::vector<float>>* ROSUnitFloatArrPub::registerPublisher(const std::string &topic_name){
    port = new ExternalInputPort<std::vector<float>>(TYPE::FloatVec);
    pub_ = nh_.advertise<std_msgs::Float32MultiArray>(topic_name, 1, true);
    return port;
}
void ROSUnitFloatArrPub::process(){
    if (port != NULL){
        std_msgs::Float32MultiArray msg;
        port->read(msg.data);
        pub_.publish(msg);
    }
}

}