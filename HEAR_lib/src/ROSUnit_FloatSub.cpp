#include "HEAR_ROS/ROSUnit_FloatSub.hpp"

namespace HEAR{

int ROSUnitFloatSub::internal_counter = 0;

ExternalOutputPort<float>* ROSUnitFloatSub::ports[ROSUnitFloatSub::capacity];

void(*ROSUnitFloatSub::callbackFunctionPointer[ROSUnitFloatSub::capacity])(const std_msgs::Float32::ConstPtr&){
    ROSUnitFloatSub::callback0,
    ROSUnitFloatSub::callback1,
    ROSUnitFloatSub::callback2,
    ROSUnitFloatSub::callback3,
    ROSUnitFloatSub::callback4,
    ROSUnitFloatSub::callback5,
    ROSUnitFloatSub::callback6,
    ROSUnitFloatSub::callback7,
    ROSUnitFloatSub::callback8,
    ROSUnitFloatSub::callback9    
};


ROSUnitFloatSub::ROSUnitFloatSub(const ros::NodeHandle& nh): nh_(nh), Block(BLOCK_ID::ROSFLOATSUB){
}
ExternalOutputPort<float>* ROSUnitFloatSub::registerSubscriber(const std::string& topic_name){
    if(internal_counter < capacity){
        ports[internal_counter] = new ExternalOutputPort<float>(TYPE::Float);
        this->sub = nh_.subscribe(topic_name, 1, callbackFunctionPointer[internal_counter]);
        internal_counter++;
        return ports[internal_counter-1];
    }
    else{
        //print error
        assert(internal_counter < capacity);
    }
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