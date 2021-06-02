#include "HEAR_ROS/ROSUnit_PointSub.hpp"

namespace HEAR{
int ROSUnitPointSub::internal_counter = 0;

ExternalOutputPort<Vector3D<float>>* ROSUnitPointSub::ports[ROSUnitPointSub::capacity];

void(*ROSUnitPointSub::callbackFunctionPointer[ROSUnitPointSub::capacity])(const geometry_msgs::Point::ConstPtr&){
    ROSUnitPointSub::callback0,
    ROSUnitPointSub::callback1,
    ROSUnitPointSub::callback2,
    ROSUnitPointSub::callback3,
    ROSUnitPointSub::callback4,
    ROSUnitPointSub::callback5,
    ROSUnitPointSub::callback6,
    ROSUnitPointSub::callback7,
    ROSUnitPointSub::callback8,
    ROSUnitPointSub::callback9    
};

ROSUnitPointSub::ROSUnitPointSub(const ros::NodeHandle& nh): nh_(nh), Block(BLOCK_ID::ROSPOINTSUB){
}
ExternalOutputPort<Vector3D<float>>* ROSUnitPointSub::registerSubscriber(const std::string& topic_name){
    if(internal_counter < capacity){
        ports[internal_counter] = new ExternalOutputPort<Vector3D<float>>(TYPE::Float3);
        this->sub = nh_.subscribe(topic_name, 1, callbackFunctionPointer[internal_counter]);
        internal_counter++;
        return ports[internal_counter-1];
    }
    else{
        //print error
        assert(internal_counter < capacity);
    }
}

void ROSUnitPointSub::callback0(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[0]->update(data);
}

void ROSUnitPointSub::callback1(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[1]->update(data);
}

void ROSUnitPointSub::callback2(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[2]->update(data);
}

void ROSUnitPointSub::callback3(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[3]->update(data);
}

void ROSUnitPointSub::callback4(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[4]->update(data);
}

void ROSUnitPointSub::callback5(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[5]->update(data);
}

void ROSUnitPointSub::callback6(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[6]->update(data);
}

void ROSUnitPointSub::callback7(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[7]->update(data);
}

void ROSUnitPointSub::callback8(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[8]->update(data);
}

void ROSUnitPointSub::callback9(const geometry_msgs::Point::ConstPtr& msg){
    Vector3D<float> data(msg->x, msg->y, msg->z);
    ports[9]->update(data);
}

}