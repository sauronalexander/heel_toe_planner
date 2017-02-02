#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
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
       std::vector<std::vector<double>> ModifiedJointAngles;
       control_msgs::FollowJointTrajectoryActionGoal ExtendTrajectory(int step, int leg);
       control_msgs::FollowJointTrajectoryActionGoal DefaultPose(int leg);
       void Debug();
       void Manual(size_t start,size_t end);
       size_t start;
       size_t end;

   protected:
       ros::NodeHandle nh;

   public:
       Reemc_Trajectory_Control(ros::NodeHandle &nh);

       virtual ~Reemc_Trajectory_Control();
       //void startTrajectory(control_msgs::JointTrajectoryGoal goal);

       void Step();
       //actionlib::SimpleClientGoalState getState();
};


#endif // CONTROLLER_H
