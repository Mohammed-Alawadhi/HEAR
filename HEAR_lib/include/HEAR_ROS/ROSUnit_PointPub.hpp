#ifndef ROSUNIT_POINTPUB_HPP
#define ROSUNIT_POINTPUB_HPP

#include "HEAR_core/DataTypes.hpp"
#include "HEAR_core/Block.hpp"
#include "HEAR_core/ExternalPort.hpp"
#include "HEAR_core/Vector3D.hpp"

#include "ros/ros.h"
#include "geometry_msgs/Point.h"

namespace HEAR{

class ROSUnitPointPub : public Block{
private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ExternalInputPort<Vector3D<float>>* port;
public:
    ROSUnitPointPub(ros::NodeHandle&);
    ExternalInputPort<Vector3D<float>>* registerPublisher(const std::string &);
    void process();
};

}

#endif