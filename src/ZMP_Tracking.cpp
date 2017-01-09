#include <iostream>
#include <thread>
#include <ros/ros.h>
#include <cstdint>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/LinkStates.h>
#include <pal_multicontact_planner/element.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>

#include <cstdint>
#include <cmath>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo/gazebo.hh>
#include <gazebo_msgs/LinkStates.h>
#include <pal_multicontact_planner/element.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <heel_toe_planner/ZMPCoordinate.h>

//Type Definition
typedef u_int64_t uint64_t;
typedef struct TorqueForce
{
    bool update;
    ros::Time time;
    Eigen::Vector3d force;
    Eigen::Vector3d torque;
} TorqueForce;

//Global Varibales
size_t left_count;
size_t right_count;
size_t states_count;
ros::Time link_state_time;
ros::Time left_sensor;
ros::Time right_sensor;
TorqueForce LeftLeg;
TorqueForce RightLeg;
bool update_left;
bool update_right;
element left6link;
element right6link;

//Publisher
ros::Publisher zmp_pub;

//Callback Functions
void Receive_Wrench_left(const geometry_msgs::WrenchStamped &msg);
void Receive_Wrench_right(const geometry_msgs::WrenchStamped &msg);
void Receive_Link_States(const gazebo_msgs::LinkStates &msg);

//Calculation Functions
void ZMP_Calculation(double & ZMP_x, double & ZMP_y);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ZMP_Tracking");
    ros::NodeHandle nh;

    ROS_INFO_STREAM("Initializing ZMP Tracking...");
    left_count = 0;
    right_count = 0;
    states_count = 0;

    LeftLeg.force.setZero();
    LeftLeg.torque.setZero();
    RightLeg.force.setZero();
    RightLeg.torque.setZero();
    LeftLeg.update = true;
    RightLeg.update = true;

    update_left = false;
    update_right = false;

    ROS_INFO_STREAM("Initializating Subscribers...");
    ros::Subscriber sub_left = nh.subscribe("left_ft", 1000, &Receive_Wrench_left);
    ros::Subscriber sub_right = nh.subscribe("right_ft", 1000, &Receive_Wrench_right);
    ros::Subscriber sub_states = nh.subscribe("gazebo/link_states", 1000, &Receive_Link_States);
    zmp_pub = nh.advertise<heel_toe_planner::ZMPCoordinate>("ZMP_Coordinate", 1000);

    ROS_INFO_STREAM("Start Tracking ZMP...");
    ros::spin();

    return 0;
}


void Receive_Wrench_left(const geometry_msgs::WrenchStamped &msg)
{
    if(!LeftLeg.update)
        return;
    left_sensor = msg.header.stamp.now();
    Eigen::Vector3d force(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    Eigen::Vector3d torque(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);

    LeftLeg.time = msg.header.stamp.now();
    LeftLeg.force = force;
    LeftLeg.torque = torque;
    update_left = true;
    left_count++;
}

void Receive_Wrench_right(const geometry_msgs::WrenchStamped &msg)
{ 
    if(!RightLeg.update)
        return;
    uint64_t time_error = msg.header.stamp.now().toNSec() - link_state_time.toNSec();
    right_sensor = msg.header.stamp.now();
    Eigen::Vector3d force(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    Eigen::Vector3d torque(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);

    RightLeg.time = msg.header.stamp.now();
    RightLeg.force = force;
    RightLeg.torque = torque;
    update_right = true;
    right_count++;
}

void Receive_Link_States(const gazebo_msgs::LinkStates &msg)
{
    if(!update_left && !update_right)
        return;
    link_state_time = ros::Time::now();

    for(size_t i=0; i<msg.name.size(); i++)
    {
        if(msg.name[i] == "reemc_full::leg_left_6_link")
            left6link.Set(msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z, msg.pose[i].orientation.w, msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z);
        else if(msg.name[i] == "reemc_full::leg_right_6_link")
            right6link.Set(msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z, msg.pose[i].orientation.w, msg.pose[i].orientation.x, msg.pose[i].orientation.y, msg.pose[i].orientation.z);
    }
    LeftLeg.update = false;
    RightLeg.update = false;

    //Transform Left Leg
    Eigen::Matrix3d transform = left6link.Get_Inverse_Porduct();
//    for(int i=0; i<3; i++)
//    {
//        for(int j=0; j<3; j++)
//            std::cout<<transform(i, j)<<", ";
//        std::cout<<std::endl;
//    }
//    ROS_INFO_STREAM(LeftLeg.force<<","<<LeftLeg.torque);
//    LeftLeg.force = transform * LeftLeg.force;
//    LeftLeg.torque = transform * LeftLeg.torque;
//    ROS_INFO_STREAM(LeftLeg.force<<","<<LeftLeg.torque);


    //Transform Right Leg
    transform = right6link.Get_Inverse_Porduct();
    RightLeg.force = transform * RightLeg.force;
    RightLeg.torque = transform * RightLeg.torque;

    double ZMP_x, ZMP_y;
    ZMP_Calculation(ZMP_x, ZMP_y);

    //Publish ZMP
    heel_toe_planner::ZMPCoordinate zmp_msg;
    zmp_msg.x = ZMP_x;
    zmp_msg.y = ZMP_y;
    zmp_msg.stamp = ros::Time::now();
    update_left = false;
    update_right = false;
    LeftLeg.update = true;
    RightLeg.update = true;
}


void ZMP_Calculation(double & ZMP_x, double & ZMP_y)
{
    uint64_t error = LeftLeg.time.toNSec() - RightLeg.time.toNSec();
    uint64_t error_left = link_state_time.toNSec() - LeftLeg.time.toNSec();
    uint64_t error_right = link_state_time.toNSec() - RightLeg.time.toNSec();

    if(abs(error) > 1000.0)
    {
        ROS_ERROR_STREAM("Left Right Sync Error Exceeds Limit: "<<error);
        return;
    }
    if(abs(error_left) > 10000000 || abs(error_right) > 10000000)
    {
        ROS_ERROR_STREAM("Left Base Sync Error: "<<error_left<<", "<<"Right Base Sync Error: "<<error_right);
        return;
    }


    double h_right = right6link.Get_Pos().z;
    double h_left = left6link.Get_Pos().z;


    ZMP_x = (RightLeg.torque(1)+LeftLeg.torque(1)+RightLeg.force(0)*h_right
                   +LeftLeg.force(0)*h_left) / (LeftLeg.force(2)+RightLeg.force(2));

    ZMP_y = -(RightLeg.torque(0)+LeftLeg.torque(0)+RightLeg.force(1)*h_right
                   +LeftLeg.force(1)*h_left) / (LeftLeg.force(2)+RightLeg.force(2));

    ROS_INFO_STREAM("ZMP Coordinates: "<<ZMP_x<<", "<<ZMP_y);
    //Which Frame?
}
