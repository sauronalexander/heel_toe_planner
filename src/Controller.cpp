#include <heel_toe_planner/Controller.h>

Reemc_Trajectory_Control::Reemc_Trajectory_Control(ros::NodeHandle &nh) : Three_Mass(nh)
{
    this->nh = nh;
    pub_left_leg = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("left_leg_controller/follow_joint_trajectory/goal", 2000);
    pub_right_leg = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("right_leg_controller/follow_joint_trajectory/goal", 2000);

}

Reemc_Trajectory_Control::~Reemc_Trajectory_Control()
{}


control_msgs::FollowJointTrajectoryActionGoal Reemc_Trajectory_Control::DefaultPose(int leg)
{
    control_msgs::FollowJointTrajectoryActionGoal goal;
    assert(this->jointNames.size());
    assert(this->Reference_Angles.size());
    assert(this->Standing_Pose.size());
    goal.goal.trajectory.points.resize(1);

    if(leg == 1) //left leg
    {
        ROS_INFO("Left Leg");
        for(size_t i=0; i<=5; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);

        goal.goal.trajectory.points[0].positions.resize(goal.goal.trajectory.joint_names.size());
        for(size_t i=0; i<6; i++)
        {
//            goal.goal.trajectory.points[0].positions[i] = this->Standing_Pose[i];
//            goal.goal.trajectory.points[0].positions[i] = this->Reference_Angles[72][i];
            goal.goal.trajectory.points[0].positions[i] = this->ModifiedJointAngles[0][i];
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
//            goal.goal.trajectory.points[0].positions[i] = this->Reference_Angles[72][i+6];
            goal.goal.trajectory.points[0].positions[i] = this->ModifiedJointAngles[0][i+6];
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
    assert(this->ModifiedJointAngles.size());
    ROS_INFO_STREAM_ONCE("Controlling "<<step<<" points...");

    if(leg == 1) //left leg
    {
        ROS_INFO("Left Leg");
        for(size_t i=0; i<=5; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        goal.goal.trajectory.points.resize(step);

        for(int j=0; j<step; j++)
        {
            goal.goal.trajectory.points[j].positions.resize(goal.goal.trajectory.joint_names.size());
            for(size_t i=0; i<6; i++)
//                goal.goal.trajectory.points[j].positions[i] = this->Reference_Angles[70+j][i];
                goal.goal.trajectory.points[j].positions[i] = this->ModifiedJointAngles[j][i];
            double index = (double) j + 1.0;
            if(j<50)
                goal.goal.trajectory.points[j].time_from_start = ros::Duration(0.05*index);
            else
                goal.goal.trajectory.points[j].time_from_start = ros::Duration(0.008*(index-50.0)+0.05*50.0);
        }
    }
    else //right leg
    {
        ROS_INFO("Right Leg");
        for(size_t i=6; i<=11; i++)
            goal.goal.trajectory.joint_names.push_back(jointNames[i]);
        goal.goal.trajectory.points.resize(step);

        for(int j=0; j<step; j++)
        {
            goal.goal.trajectory.points[j].positions.resize(goal.goal.trajectory.joint_names.size());
            for(size_t i=0; i<6; i++)
            {
//                goal.goal.trajectory.points[j].positions[i] = this->Reference_Angles[70+j][i+6];
                goal.goal.trajectory.points[j].positions[i] = this->ModifiedJointAngles[j][i+6];
//                std::cout<<goal.goal.trajectory.points[j].positions[i]<<", ";
            }
//            std::cout<<std::endl;
            double index = (double) j + 1.0;
            if(j<50)
                goal.goal.trajectory.points[j].time_from_start = ros::Duration(0.05*index);
            else
                goal.goal.trajectory.points[j].time_from_start = ros::Duration(0.006*(index-50.0)+0.05*50.0);
        }
    }

    return goal;

}

void Reemc_Trajectory_Control::Step()
{
    Manual(70, 170);
//    for(size_t j=0; j<this->ModifiedJointAngles.size(); j++)
//    {
//        std::cout<<j<<": ";
//        for(size_t i=0; i<this->ModifiedJointAngles[0].size(); i++)
//            std::cout<<this->ModifiedJointAngles[j][i]<<", ";
//        std::cout<<std::endl;
//    }
    ROS_INFO_STREAM(this->Reference_Angles.size()<<" samples in total!");
    std::cout<<this->jointNames.size()<<std::endl;
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
    std::cout<<"press a key"<<std::endl;

    getchar();
//    assert(reset_world.call(req, res));
//    ROS_INFO("RESET WORLD");
//    getchar();
    ROS_INFO("Start Walking...");
    control_msgs::FollowJointTrajectoryActionGoal goal_left_walking = ExtendTrajectory(100, 1);
    control_msgs::FollowJointTrajectoryActionGoal goal_right_walking = ExtendTrajectory(100, 2);
//    for(int i=0; i<6; i++)
//    {
        ROS_ERROR_STREAM("system time: "<<ros::Time::now().toSec());
        goal_left_walking.goal.trajectory.header.stamp = ros::Time::now()+ros::Duration(1.0);
        goal_right_walking.goal.trajectory.header.stamp = goal_left_walking.goal.trajectory.header.stamp;
        ROS_ERROR_STREAM("walking time: "<<goal_right_walking.goal.trajectory.header.stamp.toSec());
        pub_left_leg.publish(goal_left_walking);
        pub_right_leg.publish(goal_right_walking);
//    }
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

void Reemc_Trajectory_Control::Manual(size_t start,size_t end)
{
    this->start = start;
    this->end = end;
    for(size_t i=start; i<=end; i++)
    {
        Eigen::Quaternion<double> base_ori;
        base_ori.setIdentity();
        double index = (double) i - 70.0;
        if(i > 130)
            index = 205.0;
        else if(i>100)
            index = 200.0;
        double indx = 0.0;
        if(i<=130)
            indx = 0;
        if(i<150 && i > 140)
            indx= (double) i - 140.0;
        if(i>=150)
            indx = 10.0;
        if(i>150)
            indx = 160.0 - (double) i;

        if(i<=130)
            UpdateBase(x_trunk->data[i] - indx*0.015, -1.0*y_trunk->data[i] +0.01 + index*0.0002, z2+this->trunk_offset, base_ori);
        else if(i<=150)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i]-0.02, z2+this->trunk_offset, base_ori);
        else if(i<=152)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i]-0.026, z2+this->trunk_offset, base_ori);
        else if(i<=154)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i]-0.034, z2+this->trunk_offset, base_ori);
        else if(i<=156)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i]-0.044, z2+this->trunk_offset, base_ori);
        else if(i<=158)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i]-0.05, z2+this->trunk_offset, base_ori);
        else if(i<160)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.052, z2+this->trunk_offset, base_ori);
        else if(i<162)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.056, z2+this->trunk_offset, base_ori);
        else if(i<164)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.059, z2+this->trunk_offset, base_ori);
        else if(i<165)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.062, z2+this->trunk_offset, base_ori);
        else if(i<166)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.07, z2+this->trunk_offset, base_ori);
        else if(i<167)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.08, z2+this->trunk_offset, base_ori);
        else if(i<168)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.09, z2+this->trunk_offset, base_ori);
        else if(i<168)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.08, z2+this->trunk_offset, base_ori);
        else if(i<169)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.07, z2+this->trunk_offset, base_ori);
        else if(i<170)
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.062, z2+this->trunk_offset, base_ori);
        else
            UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i] -0.055, z2+this->trunk_offset, base_ori);

        //Three Masses Transformation
        Eigen::Matrix3d ori_left;
        Eigen::Matrix3d ori_right;
        pcl::PointXYZ pos_left;
        pcl::PointXYZ pos_right;
        ori_left.setZero();
        ori_right.setZero();

        if(i > 1110)//temporary hard debug
        {
            ori_left(0, 0) = cos(theta_left->data[1110]);
            ori_left(0, 2) = sin(theta_left->data[1110]);
            ori_left(1, 1) = 1.0;
            ori_left(2, 0) = -1.0*sin(theta_left->data[1110]);
            ori_left(2, 2) = cos(theta_left->data[1110]);
            ori_right(0, 0) = cos(theta_right->data[1110]);
            ori_right(0, 2) = sin(theta_right->data[1110]);
            ori_right(1, 1) = 1.0;
            ori_right(2, 0) = -1.0*sin(theta_right->data[1110]);
            ori_right(2, 2) = cos(theta_right->data[1110]);
        }
        else
        {
            ori_left(0, 0) = cos(theta_left->data[i]);
            ori_left(0, 2) = sin(theta_left->data[i]);
            ori_left(1, 1) = 1.0;
            ori_left(2, 0) = -1.0*sin(theta_left->data[i]);
            ori_left(2, 2) = cos(theta_left->data[i]);
            ori_right(0, 0) = cos(theta_right->data[i]);
            ori_right(0, 2) = sin(theta_right->data[i]);
            ori_right(1, 1) = 1.0;
            ori_right(2, 0) = -1.0*sin(theta_right->data[i]);
            ori_right(2, 2) = cos(theta_right->data[i]);
        }
        pos_left.x = x_left->data[i];
        pos_left.y = w/2.0;
        pos_left.z = z_left->data[i];
        pos_right.x = x_right->data[i];
        pos_right.y = -1.0*w/2.0;
        pos_right.z = z_right->data[i]-0.01;
        element left = element(pos_left, ori_left);
        element right = element(pos_right, ori_right);

        bool left_solve = false;
        bool right_solve = false;
        std::vector<double> jointangles;
        jointangles.resize(this->jointNames.size());
        for(size_t j=0; j<this->jointNames.size(); j++)
            jointangles[j] = 0.0;

//        std::cout<<left.Get_Pos().x<<", "<<left.Get_Pos().y<<", "<<left.Get_Pos().z<<std::endl;
//        std::cout<<right.Get_Pos().x<<", "<<right.Get_Pos().y<<", "<<right.Get_Pos().z<<std::endl;
//        std::cout<<x_trunk->data[i]<<", "<<-1.0*y_trunk->data[i] +0.02<<", "<<z2+this->trunk_offset<<std::endl;

        for(int k=0; k<3; k++)
            if(Solve(1, left, jointangles, true))
            {
                left_solve = true;
                break;
            }
        for(int k=0; k<3; k++)
            if(Solve(2, right, jointangles, true))
            {
                right_solve = true;
                break;
            }

        if(!left_solve)
            ROS_FATAL_STREAM("Left Leg IK_ERROR on "<<i<<"th iteration in Manual Mode");
        else if(!right_solve)
            ROS_FATAL_STREAM("Right Leg IK_ERROR on "<<i<<"th iteration in Manual Mode");
        else
            this->ModifiedJointAngles.push_back(jointangles);

//        for(size_t j=0; j<jointangles.size(); j++)
//            std::cout<<jointangles[i]<<", ";
//        std::cout<<std::endl;
    }


}


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

