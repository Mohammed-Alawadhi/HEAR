
#ifndef ROSUNITFLOATPUB_HPP
#define ROSUNITFLOATPUB_HPP


#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "ros/ros.h"
#include "std_msgs/Float32.h"

namespace HEAR{

class ROSUnitFloatPub : public Block{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ExternalInputPort<float>* port;
public:
    ROSUnitFloatPub(ros::NodeHandle&);
    ExternalInputPort<float>* registerPublisher(const std::string &);
    void process();
};

}

#endif