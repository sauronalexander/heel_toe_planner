#ifndef THREE_MASS_H
#define THREE_MASS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <heel_toe_planner/IK_Solver.h>
#include <pal_multicontact_planner/element.h>
#include <heel_toe_planner/COM_Generation/COM_Generation_All.h>
#include <tf/transform_broadcaster.h>
#include <cassert>
#include <sensor_msgs/JointState.h>
#include <pcl/point_cloud.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <gazebo_ros_control/default_robot_hw_sim.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/gazebo.hh>

//Serialization
#include <fstream>
#include <sstream>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/tracking.hpp>
#include <boost/serialization/vector.hpp>


typedef struct Mass_Info
{
    element pose;
    double mass;
} Mass_Info;

class Three_Mass : public IK_Solver, public COM_Generation, public hardware_interface::RobotHW
{
    private:
        Mass_Info Trunk;
        Mass_Info Left;
        Mass_Info Right;
        element base;

        std::vector<double> Q;
        bool Verify();
        void Solve_Standing_Pose();

    protected:
        double trunk_offset;
        ros::Publisher jointPub;
        tf::TransformBroadcaster br;
        sensor_msgs::JointState jointStateMsg;
        std::vector<std::vector<double> > Reference_Angles;
        std::vector<int> error_vector;
        std::vector<double> Standing_Pose;

    public:
        Three_Mass(ros::NodeHandle & nh);
        void Set_Mass(double mass_trunk, double mass_leg);
        void Generate_Reference_Angles(bool visualize);
        
        virtual ~Three_Mass();
        void Error();
        //Talk to HW
        void read();
        void write();
        //Reimplement only if needed
        //virtual bool checkForConflict() const;


};


#endif // THREE_MASS_H
