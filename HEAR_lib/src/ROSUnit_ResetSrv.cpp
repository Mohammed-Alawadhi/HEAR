#include "HEAR_ROS/ROSUnit_ResetSrv.hpp"

namespace HEAR{

ResetTrigger* ROSUnit_ResetServer::ext_trigs[ROSUnit_ResetServer::capacity];

int ROSUnit_ResetServer::internal_counter = 0;

bool (*ROSUnit_ResetServer::callback_pointers[capacity])(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&){
    ROSUnit_ResetServer::srv_callback0,
    ROSUnit_ResetServer::srv_callback1,
    ROSUnit_ResetServer::srv_callback2,
    ROSUnit_ResetServer::srv_callback3,
    ROSUnit_ResetServer::srv_callback4,
    ROSUnit_ResetServer::srv_callback5,
    ROSUnit_ResetServer::srv_callback6
};

ROSUnit_ResetServer::ROSUnit_ResetServer(ros::NodeHandle &nh) : nh_(nh){
 
}

ResetTrigger* ROSUnit_ResetServer::registerServer(const std::string &service_topic){
    if(internal_counter < capacity){
        ext_trigs[internal_counter] = new ResetTrigger;
        this->m_server = nh_.advertiseService(service_topic, ROSUnit_ResetServer::callback_pointers[internal_counter]);  
        internal_counter++;
    }
    else{
        //print error
        assert(internal_counter < capacity);
    } 
    return ext_trigs[internal_counter-1];
}

bool ROSUnit_ResetServer::srv_callback0(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    ext_trigs[0]->resetCallback();
    return true;
}

bool ROSUnit_ResetServer::srv_callback1(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    ext_trigs[1]->resetCallback();
    return true;
}

bool ROSUnit_ResetServer::srv_callback2(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    ext_trigs[2]->resetCallback();
    return true;
}

bool ROSUnit_ResetServer::srv_callback3(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    ext_trigs[3]->resetCallback();
    return true;
}

bool ROSUnit_ResetServer::srv_callback4(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    ext_trigs[4]->resetCallback();
    return true;
}

bool ROSUnit_ResetServer::srv_callback5(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    ext_trigs[5]->resetCallback();
    return true;
}

bool ROSUnit_ResetServer::srv_callback6(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res){
    ext_trigs[6]->resetCallback();
    return true;
}


}