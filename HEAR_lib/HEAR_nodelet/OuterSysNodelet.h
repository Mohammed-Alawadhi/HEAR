#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"

namespace HEAR
{
class OuterSysNodelet : public nodelet::Nodelet{

public:
    OuterSysNodelet() = default;
    virtual ~OuterSysNodelet();

private:
    const int SAT_XY_VALUE = 0.25;
    const int SAT_Z_VALUE = 0.8;
    const int FREQUENCY = 120;
    virtual void onInit();
    
    RosSystem* outer_sys;
    void loopCallback();
    
};
    
} 