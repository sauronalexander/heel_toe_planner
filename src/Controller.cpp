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

control_msgs::FollowJointTrajectoryActionGoal Reemc_Trajectory_Control::DefaultPose(int leg)
{
    control_msgs::FollowJointTrajectoryActionGoal goal;
    assert(this->jointNames.size());
    assert(this->Reference_Angles.size());
    assert(this->Standing_Pose.size());
    goal.goal.trajectory.points.resize(1);
    for(size_t i=0;i<this->Reference_Angles[72].size(); i++)
        std::cout<<this->Reference_Angles[72][i]<<std::endl;

    if(leg == 1) //left leg
    {
        ROS_INFO("Left Leg");
        for(size_t i=0; i<=5; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        //we will have two waypoints in this goal trajectory

        goal.goal.trajectory.points[0].positions.resize(goal.goal.trajectory.joint_names.size());
        for(size_t i=0; i<6; i++)
        {
//            goal.goal.trajectory.points[0].positions[i] = this->Standing_Pose[i];
            goal.goal.trajectory.points[0].positions[i] = this->Reference_Angles[72][i];
            std::cout<<goal.goal.trajectory.points[0].positions[i]<<"; ";
        }
    }
    else //right leg
    {
        ROS_INFO("Right Leg");
        for(size_t i=6; i<=11; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        goal.goal.trajectory.points[0].positions.resize(goal.goal.trajectory.joint_names.size());
        for(size_t i=0; i<6; i++)
        {
//            goal.goal.trajectory.points[0].positions[i] = this->Standing_Pose[i+6];
            goal.goal.trajectory.points[0].positions[i] = this->Reference_Angles[72][i+6];
            std::cout<<goal.goal.trajectory.points[0].positions[i]<<"; ";
        }
    }
    goal.goal.trajectory.points[0].time_from_start = ros::Duration(1.0);
    return goal;

}

control_msgs::FollowJointTrajectoryActionGoal Reemc_Trajectory_Control::ExtendTrajectory(int step, int leg)
{
    //our goal variable
    control_msgs::FollowJointTrajectoryActionGoal goal;
    assert(this->jointNames.size());
    assert(this->Reference_Angles.size());
    assert(this->Standing_Pose.size());
    ROS_INFO_STREAM_ONCE("Controlling "<<step<<" points...");

    if(leg == 1) //left leg
    {
        ROS_INFO("Left Leg");
        for(size_t i=0; i<=5; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        //we will have two waypoints in this goal trajectory
        goal.goal.trajectory.points.resize(step);

        for(int j=0; j<step; j++)
        {
            goal.goal.trajectory.points[j].positions.resize(goal.goal.trajectory.joint_names.size());
            for(size_t i=0; i<6; i++)
                goal.goal.trajectory.points[j].positions[i] = this->Reference_Angles[70+j][i];
            double index = (double) j + 1.0;
            goal.goal.trajectory.points[j].time_from_start = ros::Duration(0.01*index);
        }
    }
    else //right leg
    {
        ROS_INFO("Right Leg");
        for(size_t i=6; i<=11; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        //we will have two waypoints in this goal trajectory
        goal.goal.trajectory.points.resize(step);

        for(int j=0; j<step; j++)
        {
            goal.goal.trajectory.points[j].positions.resize(goal.goal.trajectory.joint_names.size());
            for(size_t i=0; i<6; i++)
                goal.goal.trajectory.points[j].positions[i] = this->Reference_Angles[70+j][i+6];
            double index = (double) j + 1.0;
            goal.goal.trajectory.points[j].time_from_start = ros::Duration(0.01*index);
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

    control_msgs::FollowJointTrajectoryActionGoal goal_left = DefaultPose(1);
    control_msgs::FollowJointTrajectoryActionGoal goal_right = DefaultPose(2);
//    while(ros::ok())
//        Debug();
std::cout<<this->Reference_Angles[0].size()<<std::endl;

        //Reset World and Reset Simulation
    for(int i=0; i<3; i++)
    {
        goal_left.goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(1.0);
        goal_right.goal.trajectory.header.stamp = goal_left.goal.trajectory.header.stamp;
        pub_left_leg.publish(goal_left);
        pub_right_leg.publish(goal_right);
    }


    getchar();
    ROS_INFO("Start Walking...");
    goal_left = ExtendTrajectory(5, 1);
        goal_right = ExtendTrajectory(5, 2);
        goal_left.goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(1.0);
        goal_right.goal.trajectory.header.stamp = goal_left.goal.trajectory.header.stamp;
        pub_left_leg.publish(goal_left);
        pub_right_leg.publish(goal_right);
        //ros::sleep(ros::Duration(5.0));
//    }

//        assert(reset_world.call(req, res));
//        assert(reset_simulation.call(req, res));

    // }
}

void Reemc_Trajectory_Control::Debug()
{
    std::cout<<"Number: "<<std::endl;
    int n;
    std::cin>>n;
    std::cout<<"Number: "<<n<<std::endl;

    eMatrixHom baseTf;
    baseTf.setIdentity();
    baseTf(0, 3) = 0;
    baseTf(1, 3) = 0;
    baseTf(2, 3) = 0.85;
    tf::Transform base_tf;
    pal::convert(baseTf, base_tf);
    br.sendTransform(tf::StampedTransform(base_tf, ros::Time::now(), "/world", "/base_link"));

    jointStateMsg.header.stamp = ros::Time::now();
    jointStateMsg.name = this->jointNames;
    assert(jointNames.size());
    jointStateMsg.position.resize(jointNames.size());
    jointStateMsg.position = this->Reference_Angles[n];
    jointPub.publish(jointStateMsg);
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

