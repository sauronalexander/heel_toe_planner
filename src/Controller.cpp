#include <heel_toe_planner/Controller.h>

Reemc_Trajectory_Control::Reemc_Trajectory_Control(ros::NodeHandle &nh) : Three_Mass(nh)
{
    this->nh = nh;
    pub_left_leg = nh.advertise<control_msgs::JointTrajectoryGoal>("left_leg_controller/follow_joint_trajectory/goal", 2000);
    pub_right_leg = nh.advertise<control_msgs::JointTrajectoryGoal>("right_leg_controller/follow_joint_trajectory/goal", 2000);


    traj_client_ = new TrajClient("/controller_manager/list_controllers", true);
    int k=0;
    while(!traj_client_->waitForServer(ros::Duration(1.0)))
    {
        ROS_INFO("Waiting for the joint_trajectory_action server");
        k++;
        if(k>5)
            break;
    }


}

Reemc_Trajectory_Control::~Reemc_Trajectory_Control()
{
    delete traj_client_;
}

//void Reemc_Trajectory_Control::startTrajectory(control_msgs::JointTrajectoryActionGoal goal)
//{
//    //When to start: 1s from now
//    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
//}

control_msgs::JointTrajectoryGoal Reemc_Trajectory_Control::ExtendTrajectory(int step, int leg)
{
    //our goal variable
    control_msgs::JointTrajectoryGoal goal;
    if(step == 0)
        for(size_t i=0; i<jointNames.size(); i++)
            std::cout<<jointNames[i]<<std::endl;

    if(leg == 1) //left leg
        for(size_t i=0; i<=5; i++)
            goal.trajectory.joint_names.push_back(jointNames[i]);
    else //right leg
        for(size_t i=6; i<=11; i++)
            goal.trajectory.joint_names.push_back(jointNames[i]);

    //we will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);

    if(step == 0)
    {
        //First trajectory point
        //Position
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(jointNames.size());
        for(size_t i=0; i<jointNames.size(); i++)
            goal.trajectory.points[ind].positions[i] = 0.05;

        //Velocity
        goal.trajectory.points[ind].velocities.resize(jointNames.size());
        for(size_t i=0; i<jointNames.size(); i++)
            goal.trajectory.points[ind].velocities[i] = 0.0;
        goal.trajectory.points[ind].time_from_start = ros::Duration(0.0);
    }



    //Second trajectory point
    //Position
    if(step == 1)
    {
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(jointNames.size());
        for(size_t i=0; i<jointNames.size(); i++)
            goal.trajectory.points[ind].positions[i] = 0.1;

        //Velocity
        goal.trajectory.points[ind].velocities.resize(jointNames.size());
        for(size_t i=0; i<jointNames.size(); i++)
            goal.trajectory.points[ind].velocities[i] = 0.0;

        goal.trajectory.points[ind].time_from_start = ros::Duration(0.0);
    }

    return goal;

}

void Reemc_Trajectory_Control::Step()
{
    for(int i=0; i<2; i++)
    {
        control_msgs::JointTrajectoryGoal goal_left = ExtendTrajectory(i, 1);
        control_msgs::JointTrajectoryGoal goal_right = ExtendTrajectory(i, 2);
        goal_left.trajectory.header.stamp = ros::Time::now();
        goal_right.trajectory.header.stamp = ros::Time::now();
        pub_left_leg.publish(goal_left);
        pub_right_leg.publish(goal_right);
        ros::Duration(1.0).sleep();

    }
}

//actionlib::SimpleClientGoalState Reemc_Trajectory_Control::getState()
//{
//    //return traj_client_->getState();
//}
