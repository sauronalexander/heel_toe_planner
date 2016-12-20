#include <heel_toe_planner/Controller.h>

Reemc_Trajectory_Control::Reemc_Trajectory_Control(ros::NodeHandle &nh) : Three_Mass(nh)
{
    this->nh = nh;
    pub_left_leg = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("left_leg_controller/follow_joint_trajectory/goal", 2000);
    pub_right_leg = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("right_leg_controller/follow_joint_trajectory/goal", 2000);


    traj_client_ = new TrajClient("/controller_manager/list_controllers", true);

//    while(!traj_client_->waitForServer(ros::Duration(1.0)))
//    {
//        ROS_INFO("Waiting for the joint_trajectory_action server");
//        k++;
//        if(k>5)
//            break;
//    }


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

control_msgs::FollowJointTrajectoryActionGoal Reemc_Trajectory_Control::ExtendTrajectory(int step, int leg)
{
    //our goal variable
    control_msgs::FollowJointTrajectoryActionGoal goal;
    assert(this->jointNames.size());
    assert(this->Reference_Angles.size());
    ROS_INFO_STREAM("Controlling "<<step<<" points...");
    /*
    if(leg == 1) //left leg
    {
        for(size_t i=0; i<=5; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        //we will have two waypoints in this goal trajectory
        goal.goal.trajectory.points.resize(1);

        int ind = 0;
        goal.goal.trajectory.points[ind].positions.resize(goal.goal.trajectory.joint_names.size());
        for(size_t i=0; i<5; i++)
            goal.goal.trajectory.points[ind].positions[i] = this->Reference_Angles[step][i];

        //Velocity
        goal.goal.trajectory.points[ind].velocities.resize(goal.goal.trajectory.joint_names.size());
        for(size_t i=0; i<5; i++)
            goal.goal.trajectory.points[ind].velocities[i] = 0.0;
        goal.goal.trajectory.points[ind].time_from_start = ros::Duration(0.1);
    }
    else //right leg
    {
        for(size_t i=6; i<=11; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        //we will have two waypoints in this goal trajectory
        goal.goal.trajectory.points.resize(1);

        int ind = 0;
        goal.goal.trajectory.points[ind].positions.resize(goal.goal.trajectory.joint_names.size());
        for(size_t i=0; i<5; i++)
            goal.goal.trajectory.points[ind].positions[i] = this->Reference_Angles[step][i+6];

        //Velocity
        goal.goal.trajectory.points[ind].velocities.resize(goal.goal.trajectory.joint_names.size());
        for(size_t i=0; i<5; i++)
            goal.goal.trajectory.points[ind].velocities[i] = 0.0;
        goal.goal.trajectory.points[ind].time_from_start = ros::Duration(0.01);
    }
*/
    if(leg == 1) //left leg
    {
        for(size_t i=0; i<=5; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        //we will have two waypoints in this goal trajectory
        goal.goal.trajectory.points.resize(this->Reference_Angles.size());


        for(int j=0; j<step; j++)
        {
            goal.goal.trajectory.points[j].positions.resize(goal.goal.trajectory.joint_names.size());
            for(size_t i=0; i<5; i++)
                goal.goal.trajectory.points[j].positions[i] = this->Reference_Angles[j][i];
            double index = (double) j;
            goal.goal.trajectory.points[j].time_from_start = ros::Duration(0.1*index);
        }


    }
    else //right leg
    {
        for(size_t i=6; i<=11; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        //we will have two waypoints in this goal trajectory
        goal.goal.trajectory.points.resize(this->Reference_Angles.size());

        for(int j=0; j<step; j++)
        {
            goal.goal.trajectory.points[j].positions.resize(goal.goal.trajectory.joint_names.size());
            for(size_t i=0; i<5; i++)
                goal.goal.trajectory.points[j].positions[i] = this->Reference_Angles[j][i+6];
            double index = (double) j;
            goal.goal.trajectory.points[j].time_from_start = ros::Duration(0.1*index);
        }

    }



    return goal;

}

void Reemc_Trajectory_Control::Step()
{
    ROS_INFO_STREAM(this->Reference_Angles.size()<<" samples in total!");
    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    ros::ServiceClient reset_world = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    ros::ServiceClient reset_simulation = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation");
    while(ros::ok())
    {
        control_msgs::FollowJointTrajectoryActionGoal goal_left = ExtendTrajectory(20, 1);
        control_msgs::FollowJointTrajectoryActionGoal goal_right = ExtendTrajectory(20, 2);
        goal_left.goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(1.0);
        goal_right.goal.trajectory.header.stamp = goal_left.goal.trajectory.header.stamp;
        pub_left_leg.publish(goal_left);
        pub_right_leg.publish(goal_right);
        //Reset World and Reset Simulation
        assert(reset_world.call(req, res));
        assert(reset_simulation.call(req, res));
        ros::Duration(10).sleep();

    }
}

//actionlib::SimpleClientGoalState Reemc_Trajectory_Control::getState()
//{
//    //return traj_client_->getState();
//}


//leg_left_1_joint
//leg_left_2_joint
//leg_left_3_joint
//leg_left_4_joint
//leg_left_5_joint
//leg_left_6_joint
//leg_right_1_joint
//leg_right_2_joint
//leg_right_3_joint
//leg_right_4_joint
//leg_right_5_joint
//leg_right_6_joint

