#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "HEAR_ROS/RosSystem.hpp"

#define BIG_HEXA

namespace HEAR
{
class ActuationSysNodelet : public nodelet::Nodelet{

public:
    ActuationSysNodelet() = default;
    virtual ~ActuationSysNodelet();

private:
    const int FREQUENCY = 200;

    virtual void onInit();
    
    RosSystem* actuation_sys;
    ros::Subscriber _hb_sub;
};
    
} 