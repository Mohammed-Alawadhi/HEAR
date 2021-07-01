#include "ros/ros.h"
#include "nodelet/nodelet.h"

#include "HEAR_ROS/RosSystem.hpp"
#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"
#include "HEAR_ROS/RosUnit_MRFTSwitchSrv.hpp"

namespace HEAR
{
class OuterSysNodelet : public nodelet::Nodelet{

public:
    OuterSysNodelet() = default;
    virtual ~OuterSysNodelet();

private:
    const float SAT_XY_VALUE = 0.25;
    const float SAT_Z_VALUE = 0.9;
    const float BIAS_Z_VALUE = 0.4;
    const int FREQUENCY = 120;
    virtual void onInit();
    
    RosSystem* outer_sys;    
    ROSUnit_PoseProvider* providers;
    ROSUnit_MRFTSwitchSrv* trig_srv_x;
    ROSUnit_MRFTSwitchSrv* trig_srv_z;
    ROSUnit_MRFTSwitchSrv* trig_srv_y;

};
    
} 