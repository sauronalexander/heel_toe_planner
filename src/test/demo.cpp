#include <ros/ros.h>
#include <rbdl/rbdl.h>
#include <pal_multicontact_planner/element.h>
#include <heel_toe_planner/Three_Mass.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/JointState.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/msgs/joint_cmd.pb.h>
#include <gazebo/gazebo_config.h>
#include <gazebo/Server.hh>
#include <gazebo/math/gzmath.hh>
#include <controller_manager/controller_manager.h>
#include <gazebo_msgs/SetJointProperties.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/SetJointTrajectory.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/SetModelState.h>


//#include <gazebo/set_joint_properties.h>



int main(int argc, char ** argv)
{
    
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("DEMO");
    Three_Mass MyRobot(nh);
//    controller_manager::ControllerManager cm(&MyRobot, nh);

    ros::ServiceClient srvClient= nh.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
    gazebo_msgs::SetLinkState::Request req;
    gazebo_msgs::SetLinkState::Response resp;
    req.link_state.link_name = "leg_left_4_link";

    req.link_state.pose.position.x = 0.2;
    req.link_state.pose.position.y = 0.2;
    req.link_state.pose.position.z = 0.2;

    ros::ServiceClient srvClient2 = nh.serviceClient<gazebo_msgs::SetModelState>("gazebo/set_model_state");
    gazebo_msgs::SetModelState::Request req1;
    gazebo_msgs::SetModelState::Response resp1;
    req1.model_state.model_name = "leg_left_4_link";
    req1.model_state.pose.position.x = 0.2;
    req1.model_state.pose.position.y = 0.2;
    req1.model_state.pose.position.z = 0.2;

    bool success = srvClient.call(req, resp);

    if(success)
    {
        ROS_INFO_STREAM("Set "<<req.link_state.link_name<<" to value.");
    }
    else
        ROS_ERROR_STREAM("Failed to set joint angles.");

    success = srvClient2.call(req1, resp1);
    if(success)
    {
        ROS_INFO_STREAM("Set "<<req1.model_state.model_name<<" to value.");
    }
    else
        ROS_ERROR_STREAM("Failed to set joint angles.");


    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0);
    while(ros::ok())
    {
        const ros::Time time = ros::Time::now();
        const ros::Duration period = time-prev_time;

        MyRobot.read();
//        cm.update(time, period);
        MyRobot.write();

        rate.sleep();
    }

        
    element temp();
    ros::ServiceClient JointPublisher;
    //gazebo::

//    ros::Publisher jointPub= nh.advertise<sensor_msgs::JointState>("/rrbot/joint_states", 100);
//    sensor_msgs::JointState jointStateMsg;
//    jointStateMsg.header.stamp = ros::Time::now();

//    jointStateMsg.name.push_back("joint1");
//    jointStateMsg.name.push_back("joint2");
//    jointStateMsg.position.resize(2);
//    while(ros::ok())
//    {
//        jointStateMsg.position[0] = 1.5;
//        jointStateMsg.position[1] = 1.5;
//        jointPub.publish(jointStateMsg);
//        ros::Duration(1.0).sleep();
//    }

    return 0;
}
