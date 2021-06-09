// TODO: add printing pub sub connections
// TODO: refactoring the ros services

#ifndef ROSSYSTEM_HPP
#define ROSSYSTEM_HPP

#include "System.hpp"
#include "HEAR_ROS/ROSUnit_Pub.hpp"
#include "HEAR_ROS/ROSUnit_FloatArrPub.hpp"
#include "HEAR_ROS/ROSUnit_FloatPub.hpp"
#include "HEAR_ROS/ROSUnit_PointPub.hpp"
#include "HEAR_ROS/ROSUnit_ResetSrv.hpp"
#include "HEAR_ROS/ROSUnit_UpdateContSrv.hpp"
#include "HEAR_ROS/ROSUnit_BoolSrv.hpp"
#include "HEAR_ROS/ROSUnit_Sub.hpp"
#include "HEAR_ROS/ROSUnit_PointSub.hpp"
#include "HEAR_ROS/ROSUnit_FloatSub.hpp"
#include <ros/ros.h>

namespace HEAR{

class RosSystem : public System {
public:
    RosSystem(ros::NodeHandle& nh, ros::NodeHandle& pnh, const int frequency, const std::string& sys_name ) : nh_(nh), pnh_(pnh), System(frequency, sys_name){} 
    ROSUnit_Sub* createSub(TYPE d_type, std::string topic_name);
    template <class T> void createPub(TYPE d_type, std::string topic_name, OutputPort<T>* src_port);
    void createPub(std::string topic_name, OutputPort<float>* src_port);
    template <class T> ROSUnit_Sub* createSub(TYPE d_type, std::string topic_name, InputPort<T>* dest_port);
    template <class T> void connectSub(ROSUnit_Sub* sub, InputPort<T>* port);
    ExternalTrigger* createResetTrigger(std::string topic);
    ExternalTrigger* createResetTrigger(std::string topic, Block* dest_block);
    ExternalTrigger* createUpdateTrigger(UPDATE_MSG_TYPE type, std::string topic);
    ExternalTrigger* createUpdateTrigger(UPDATE_MSG_TYPE type, std::string topic, Block* dest_block);
    void run();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::vector<ROSUnit_Pub*> _ros_pubs;
    std::vector<ROSUnit_Sub*> _ros_subs;
    std::vector<std::string> ros_pub_names, ros_sub_names;
    std::vector<std::pair<int, Port*>> pub_cons, sub_cons;
    template <class T> void connectPub(ROSUnit_Pub* pub, OutputPort<T>* port);
    int pub_counter = 0;
    int sub_counter = 0;
};

template <class T> 
void RosSystem::connectSub(ROSUnit_Sub* sub, InputPort<T>* port){
    port->connect((OutputPort<T>*)sub->getOutputPort<T>());
    sub_cons.push_back(std::make_pair(sub->getID(), port));
}

ROSUnit_Sub* RosSystem::createSub(TYPE d_type, std::string topic_name){
    ROSUnit_Sub* sub;
    switch(d_type){
        case TYPE::Float3 :
            sub = new ROSUnitPointSub(pnh_, topic_name, sub_counter++);
            break;
        case TYPE::Float :
        default:
            sub = new ROSUnitFloatSub(nh_, topic_name, sub_counter++);
            break;
    }
    _ros_subs.push_back(sub);
    return sub;
}

template <class T>
ROSUnit_Sub* RosSystem::createSub(TYPE d_type, std::string topic_name, InputPort<T>* dest_port){
    auto sub = this->createSub(d_type, topic_name);    
    this->connectSub(sub, dest_port);
    return sub;
}

template <class T> 
void RosSystem::connectPub(ROSUnit_Pub* pub, OutputPort<T>* port){
    ((InputPort<T>*)pub->getInputPort<T>())->connect(port);
    pub_cons.push_back(std::make_pair(pub->getID(), port));
}

template <class T>
void RosSystem::createPub(TYPE d_type, std::string topic_name, OutputPort<T>* src_port){
    ROSUnit_Pub* pub;
    switch(d_type){
        case TYPE::Float3 :
            pub = new ROSUnitPointPub(pnh_, topic_name, pub_counter++);
            break;
        case TYPE::FloatVec :
            pub = new ROSUnitFloatArrPub(pnh_, topic_name, pub_counter++);
            break;
        case TYPE::Float :
        default:
            pub = new ROSUnitFloatPub(nh_, topic_name, pub_counter++);
            break;
    }
    this->connectPub<T>(pub, src_port);
    _ros_pubs.push_back(pub);
}

void RosSystem::createPub(std::string topic_name, OutputPort<float>* src_port){
    this->createPub<float>(TYPE::Float, topic_name, src_port);
}

ExternalTrigger* RosSystem::createResetTrigger(std::string topic){
    auto srv = new ROSUnit_ResetServer(pnh_);
    auto trig = srv->registerServer(topic);
    this->addExternalTrigger(trig, topic);
    return trig;
}

ExternalTrigger* RosSystem::createResetTrigger(std::string topic, Block* dest_block){
    auto trig = this->createResetTrigger(topic);
    this->connectExternalTrigger(trig, dest_block);
    return trig;
}

ExternalTrigger* RosSystem::createUpdateTrigger(UPDATE_MSG_TYPE type, std::string topic){
    //TODO make class for ROSUnit_Srv
    ExternalTrigger* trig;
    switch(type){
        case UPDATE_MSG_TYPE::PID_UPDATE :
            auto srv = new ROSUnit_UpdateContSrv(pnh_);
            trig = srv->registerServer(topic);
            break;
        case UPDATE_MSG_TYPE::BOOL_MSG :
            auto srv = new ROSUnit_BoolServer(pnh_);
            trig = srv->registerServer(topic);
            break;
        default:
            return NULL;
    }
    this->addExternalTrigger(trig, topic);

    return trig;
}

ExternalTrigger* RosSystem::createUpdateTrigger(UPDATE_MSG_TYPE type, std::string topic, Block* dest_block){
    auto trig = this->createUpdateTrigger(type, topic);
    this->connectExternalTrigger(trig, dest_block);

    return trig;
}

void RosSystem::run(){
    
    this->loop();
    
    for(const auto& ros_pub : _ros_pubs){
        ros_pub->process();
    }

}


}

#endif