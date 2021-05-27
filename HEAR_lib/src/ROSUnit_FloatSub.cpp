#include "HEAR_ROS/ROSUnit_FloatSub.hpp"

//HEAR::ExternalOutputPort<float>* HEAR::ROSUnitFloatSub::op_0 = new HEAR::ExternalOutputPort<float>(TYPE::Float);

int HEAR::ROSUnitFloatSub::internal_counter = 0;

HEAR::ExternalOutputPort<float>* HEAR::ROSUnitFloatSub::ports[HEAR::ROSUnitFloatSub::capacity] = {
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float),
    new HEAR::ExternalOutputPort<float>(TYPE::Float)
    };

void(*HEAR::ROSUnitFloatSub::callbackFunctionPointer[HEAR::ROSUnitFloatSub::capacity])(const std_msgs::Float32::ConstPtr&){
    HEAR::ROSUnitFloatSub::callback0,
    HEAR::ROSUnitFloatSub::callback1,
    HEAR::ROSUnitFloatSub::callback2,
    HEAR::ROSUnitFloatSub::callback3,
    HEAR::ROSUnitFloatSub::callback4,
    HEAR::ROSUnitFloatSub::callback5,
    HEAR::ROSUnitFloatSub::callback6,
    HEAR::ROSUnitFloatSub::callback7,
    HEAR::ROSUnitFloatSub::callback8,
    HEAR::ROSUnitFloatSub::callback9    
};

namespace HEAR{
ROSUnitFloatSub::ROSUnitFloatSub(const ros::NodeHandle& nh): nh_(nh), Block(BLOCK_ID::ROSFLOATSUB){
}
ExternalOutputPort<float>* ROSUnitFloatSub::registerSubscriber(const std::string& topic_name){
    this->sub = nh_.subscribe(topic_name, 1, callbackFunctionPointer[internal_counter]);
    internal_counter++;
    return ports[internal_counter-1];
}

void ROSUnitFloatSub::callback0(const std_msgs::Float32::ConstPtr& msg){
    ports[0]->update(msg->data);
}

void ROSUnitFloatSub::callback1(const std_msgs::Float32::ConstPtr& msg){
    ports[1]->update(msg->data);
}

void ROSUnitFloatSub::callback2(const std_msgs::Float32::ConstPtr& msg){
    ports[2]->update(msg->data);
}

void ROSUnitFloatSub::callback3(const std_msgs::Float32::ConstPtr& msg){
    ports[3]->update(msg->data);
}

void ROSUnitFloatSub::callback4(const std_msgs::Float32::ConstPtr& msg){
    ports[4]->update(msg->data);
}

void ROSUnitFloatSub::callback5(const std_msgs::Float32::ConstPtr& msg){
    ports[5]->update(msg->data);
}

void ROSUnitFloatSub::callback6(const std_msgs::Float32::ConstPtr& msg){
    ports[6]->update(msg->data);
}

void ROSUnitFloatSub::callback7(const std_msgs::Float32::ConstPtr& msg){
    ports[7]->update(msg->data);
}

void ROSUnitFloatSub::callback8(const std_msgs::Float32::ConstPtr& msg){
    ports[8]->update(msg->data);
}

void ROSUnitFloatSub::callback9(const std_msgs::Float32::ConstPtr& msg){
    ports[9]->update(msg->data);
}


}