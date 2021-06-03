#include "HEAR_ROS/ROSUnit_BoolSrv.hpp"

namespace HEAR{

UpdateTrigger* ROSUnit_BoolServer::ext_trigs[ROSUnit_BoolServer::capacity];

int ROSUnit_BoolServer::internal_counter = 0;

bool (*ROSUnit_BoolServer::callback_pointers[capacity])(hear_msgs::set_bool::Request&, hear_msgs::set_bool::Response&){
    ROSUnit_BoolServer::srv_callback0,
    ROSUnit_BoolServer::srv_callback1,
    ROSUnit_BoolServer::srv_callback2,
    ROSUnit_BoolServer::srv_callback3,
    ROSUnit_BoolServer::srv_callback4,
};

ROSUnit_BoolServer::ROSUnit_BoolServer(ros::NodeHandle &nh) : nh_(nh){
 
}

UpdateTrigger* ROSUnit_BoolServer::registerServer(const std::string &service_topic){
    if(internal_counter < capacity){
        ext_trigs[internal_counter] = new UpdateTrigger;
        this->m_server = nh_.advertiseService(service_topic, ROSUnit_BoolServer::callback_pointers[internal_counter]);  
        internal_counter++;
    }
    else{
        //print error
        assert(internal_counter < capacity);
    } 
    return ext_trigs[internal_counter-1];
}

bool ROSUnit_BoolServer::srv_callback0(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res){
    BoolMsg msg;
    msg.data = req.data;
    ext_trigs[0]->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

bool ROSUnit_BoolServer::srv_callback1(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res){
    BoolMsg msg;
    msg.data = req.data;
    ext_trigs[1]->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

bool ROSUnit_BoolServer::srv_callback2(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res){
    BoolMsg msg;
    msg.data = req.data;
    ext_trigs[2]->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

bool ROSUnit_BoolServer::srv_callback3(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res){
    BoolMsg msg;
    msg.data = req.data;
    ext_trigs[3]->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

bool ROSUnit_BoolServer::srv_callback4(hear_msgs::set_bool::Request& req, hear_msgs::set_bool::Response& res){
    BoolMsg msg;
    msg.data = req.data;
    ext_trigs[4]->UpdateCallback((UpdateMsg*)&msg);
    return true;
}

}