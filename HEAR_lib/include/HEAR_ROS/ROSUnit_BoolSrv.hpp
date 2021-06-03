#ifndef ROSUNIT_BOOLSRV_HPP
#define ROSUNIT_BOOLSRV_HPP

#include <ros/ros.h>
#include <hear_msgs/set_bool.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_BoolServer {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    static const int capacity = 5; 
    static int internal_counter;
    static UpdateTrigger* ext_trigs[capacity];
    static bool (*callback_pointers[capacity])(hear_msgs::set_bool::Request&, hear_msgs::set_bool::Response&);
    static bool srv_callback0(hear_msgs::set_bool::Request&, hear_msgs::set_bool::Response&);
    static bool srv_callback1(hear_msgs::set_bool::Request&, hear_msgs::set_bool::Response&);
    static bool srv_callback2(hear_msgs::set_bool::Request&, hear_msgs::set_bool::Response&);
    static bool srv_callback3(hear_msgs::set_bool::Request&, hear_msgs::set_bool::Response&);
    static bool srv_callback4(hear_msgs::set_bool::Request&, hear_msgs::set_bool::Response&);
public:
    ROSUnit_BoolServer(ros::NodeHandle&);
    UpdateTrigger* registerServer(const std::string&);
    
};

}

#endif