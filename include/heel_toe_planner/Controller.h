#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/JointTrajectoryAction.h>
#include <rbdl/addons/rbdlUrdfParser.h>
#include <heel_toe_planner/Three_Mass.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajClient;

class Reemc_Trajectory_Control: public Three_Mass
{
   private:
       TrajClient* traj_client_;
       ros::Publisher pub_left_leg;
       ros::Publisher pub_right_leg;

   protected:
       ros::NodeHandle nh;

   public:
       Reemc_Trajectory_Control(ros::NodeHandle &nh);
       virtual ~Reemc_Trajectory_Control();
       //void startTrajectory(control_msgs::JointTrajectoryGoal goal);
       control_msgs::FollowJointTrajectoryActionGoal ExtendTrajectory(int step, int leg);
       void Step();
       //actionlib::SimpleClientGoalState getState();
};


#endif // CONTROLLER_H
