
#ifndef ROSUNIT_RESETSRV_HPP
#define ROSUNIT_RESETSRV_HPP

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <string>

#include "HEAR_core/ExternalTrigger.hpp"

namespace HEAR{
class ROSUnit_ResetServer {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer m_server;
    static const int capacity = 7; 
    static int internal_counter;
    static ResetTrigger* ext_trigs[capacity];
    static bool (*callback_pointers[capacity])(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
    static bool srv_callback0(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
    static bool srv_callback1(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
    static bool srv_callback2(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
    static bool srv_callback3(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
    static bool srv_callback4(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
    static bool srv_callback5(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
    static bool srv_callback6(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);
public:
    ROSUnit_ResetServer(ros::NodeHandle&);
    ResetTrigger* registerServer(const std::string&);
    
 
};

}

#endif