#include "ros/ros.h"
#include "nodelet/nodelet.h"

namespace HEAR
{
class OuterSysNodelet : public nodelet::Nodelet{

public:
    OuterSysNodelet() = default;

private:
    const int SAT_XY_VALUE = 0.25;
    const int SAT_Z_VALUE = 0.8;
    const int FREQUENCY = 120;
    virtual void onInit();
    
    void loopCallback();
    ros::Timer timer_;
};
    
} 