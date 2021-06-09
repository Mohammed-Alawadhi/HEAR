#include "ros/ros.h"
#include "nodelet/nodelet.h"

namespace HEAR
{
class OuterSysNodelet : public nodelet::Nodelet{

public:
    OuterSysNodelet();

private:
    virtual void onInit();
    
    void loopCallback();
    ros::Timer timer_;
};
    
} 