#ifndef ROSUNIT_FLOATARRPUB_HPP
#define ROSUNIT_FLOATARRPUB_HPP

#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

#include <vector>

namespace HEAR{

class ROSUnitFloatArrPub : public Block{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ExternalInputPort<std::vector<float>>* port;
public:
    ROSUnitFloatArrPub(ros::NodeHandle&);
    ExternalInputPort<std::vector<float>>* registerPublisher(const std::string &);
    void process();
};

}

#endif