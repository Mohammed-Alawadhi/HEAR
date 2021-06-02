#include "HEAR_ROS/ROSUnit_PointPub.hpp"

namespace HEAR{

ROSUnitPointPub::ROSUnitPointPub(ros::NodeHandle& nh) : nh_(nh), Block(BLOCK_ID::ROSFLOATPUB){
   
}

ExternalInputPort<Vector3D<float>>* ROSUnitPointPub::registerPublisher(const std::string &topic_name){
    port = new ExternalInputPort<Vector3D<float>>(TYPE::Float3);
    pub_ = nh_.advertise<geometry_msgs::Point>(topic_name, 1, true);
    return port;
}
void ROSUnitPointPub::process(){
    if (port != NULL){
        geometry_msgs::Point msg;
        Vector3D<float> data;
        port->read(data);
        msg.x = data.x; msg.y = data.y, msg.z = data.z;
        pub_.publish(msg);
    }
}

}