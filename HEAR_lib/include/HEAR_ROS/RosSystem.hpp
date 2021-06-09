#ifndef ROSSYSTEM_HPP
#define ROSSYSTEM_HPP

#include "System.hpp"
#include "HEAR_ROS/ROSUnitPub.hpp"
#include "HEAR_ROS/ROSUnit_FloatArrPub.hpp"
#include "HEAR_ROS/ROSUnit_FloatPub.hpp"
#include "HEAR_ROS/ROSUnit_PointPub.hpp"
#include "HEAR_ROS/ROSUnit_ResetSrv.hpp"
#include "HEAR_ROS/ROSUnit_UpdateContSrv.hpp"
#include "HEAR_ROS/ROSUnit_BoolSrv.hpp"
#include <ros/ros.h>

namespace HEAR{

class RosSystem : public System {
public:
    RosSystem(ros::NodeHandle& nh, ros::NodeHandle pnh, const int frequency, const std::string& sys_name ) : nh_(nh), pnh_(pnh), System(frequency, sys_name){} 
    Block* createSub(TYPE d_type, std::string topic_name);
    template <class T> void createPub(TYPE d_type, std::string topic_name, OutputPort<T>* src_port);
    void createPub(std::string topic_name, OutputPort<float>* src_port);
    template <class T> Block* createSub(TYPE d_type, std::string topic_name, InputPort<T>* src_port);
    template <class T> void connectSub();
    ExternalTrigger* createResetTrigger(std::string topic);
    ExternalTrigger* createResetTrigger(std::string topic, Block* dest_block);
    ExternalTrigger* createUpdateTrigger(UPDATE_MSG_TYPE type, std::string topic);
    ExternalTrigger* createUpdateTrigger(UPDATE_MSG_TYPE type, std::string topic, Block* dest_block);
    void run();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::vector<ROSUnit_Pub*> _ros_pubs;
    std::vector<std::string> ros_pub_names;
    std::vector<std::pair<int, int>> pub_cons;
    template <class T> void connectPub(ROSUnit_Pub* pub, OutputPort<T>* port);
    int pub_counter = 0;
};

template <class T> 
void RosSystem::connectPub(ROSUnit_Pub* pub, OutputPort<T>* port){
    ((InputPort<T>*)pub->getInputPort<T>())->connect(port);
    pub_cons.push_back(std::make_pair(pub->getID(), port->getPortUID()));
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