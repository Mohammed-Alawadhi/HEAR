#include "HEAR_ROS/ROSUnit_PoseProvider.hpp"

namespace HEAR{


ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::opti_pos_port;
ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::opti_ori_port;
ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::imu_ori_port;
ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::imu_acc_port;
ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::imu_angular_rt_port;

tf2::Matrix3x3 ROSUnit_PoseProvider::rot_offset;
tf2::Vector3 ROSUnit_PoseProvider::trans_offset;

ROSUnit_PoseProvider::ROSUnit_PoseProvider(ros::NodeHandle& nh): nh_(nh), Block(BLOCK_ID::ROSPOSPROV){
    m_server = nh_.advertiseService("set_height_offset", ROSUnit_PoseProvider::srv_callback);
    rot_offset.setRPY(0.0, 0.0, M_PI/2.0);

}


std::vector<ExternalOutputPort<Vector3D<float>>*> ROSUnit_PoseProvider::registerOptiPose(std::string t_name){
    opti_pos_port = new ExternalOutputPort<Vector3D<float>>(TYPE::Float3);
    opti_ori_port = new ExternalOutputPort<Vector3D<float>>(TYPE::Float3);
    opti_sub = nh_.subscribe(t_name, 10, ROSUnit_PoseProvider::callback_opti_pose, ros::TransportHints().tcpNoDelay());
    return std::vector<ExternalOutputPort<Vector3D<float>>*>{opti_pos_port, opti_ori_port};
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuOri(std::string t_name){
    imu_ori_port = new ExternalOutputPort<Vector3D<float>>(TYPE::Float3);
    xsens_ori_sub = nh_.subscribe(t_name, 10, ROSUnit_PoseProvider::callback_ori, ros::TransportHints().tcpNoDelay());
    return imu_ori_port;
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuAngularRate(std::string t_name){
    imu_angular_rt_port = new ExternalOutputPort<Vector3D<float>>(TYPE::Float3);
    xsens_ang_vel_sub = nh_.subscribe(t_name, 10, ROSUnit_PoseProvider::callback_angular_vel, ros::TransportHints().tcpNoDelay());
    return imu_angular_rt_port;
}

ExternalOutputPort<Vector3D<float>>* ROSUnit_PoseProvider::registerImuAcceleration(std::string t_name){
    imu_acc_port = new ExternalOutputPort<Vector3D<float>>(TYPE::Float3);
    xsens_free_acc_sub = nh_.subscribe(t_name, 10, ROSUnit_PoseProvider::callback_free_acc, ros::TransportHints().tcpNoDelay());
    return imu_acc_port;
}

bool ROSUnit_PoseProvider::srv_callback(hear_msgs::set_float::Request& req, hear_msgs::set_float::Response& res) {
    trans_offset.setZ(req.data);
    return true;
}

void ROSUnit_PoseProvider::callback_opti_pose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    auto pos = tf2::Vector3({msg->pose.position.x, msg->pose.position.y, msg->pose.position.z});
    auto calib_pos = rot_offset*pos - trans_offset;

    Vector3D<float> vec = {(float)calib_pos.x(), (float)calib_pos.y(), (float)calib_pos.z()};

    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w ));

    tf2Scalar yaw, pitch, roll;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec_ori = {(float)roll, (float)pitch, (float)yaw};

    opti_pos_port->update(vec);
    opti_ori_port->update(vec_ori);
}

void ROSUnit_PoseProvider::callback_ori(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
    
    auto R_mat = tf2::Matrix3x3(tf2::Quaternion(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w));

    tf2Scalar yaw, roll, pitch;
    R_mat.getEulerYPR(yaw, pitch, roll);

    Vector3D<float> vec = {(float)roll, (float)pitch, (float)yaw};
    imu_ori_port->update(vec);
}

void ROSUnit_PoseProvider::callback_angular_vel(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    Vector3D<float> vec = {(float)msg->vector.x, (float)msg->vector.x, (float)msg->vector.z};

    imu_angular_rt_port->update(vec);
}

void ROSUnit_PoseProvider::callback_free_acc(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    Vector3D<float> vec = {(float)msg->vector.x, (float)msg->vector.x, (float)msg->vector.z};

    imu_acc_port->update(vec);
}

}