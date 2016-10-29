#include <heel_toe_planner/Three_Mass.h>

Three_Mass::Three_Mass(ros::NodeHandle & nh) : IK_Solver(nh), COM_Generation()
{
    //Initialize Publisher
    jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);
    //Initialize Joint State Message
    jointStateMsg.header.stamp = ros::Time::now();
    jointStateMsg.name = this->jointNames;
    assert(jointNames.size());
    jointStateMsg.position.resize(jointNames.size());
    for(size_t i=0; i<jointNames.size(); i++)
    {
        //Initialize Joint Angles
        Q.push_back(0.0);
        jointStateMsg.position[i] = 0.0;
    }
    jointPub.publish(jointStateMsg);

    base.Set(0, trunk_offset, 0, 1, 0, 0, 0);

    Eigen::VectorXd joint_value(jointNames.size());
    joint_value.setZero();
    std::vector<Eigen::Vector3d> position_set;
    for(int i=1; i<=6; i++)
    {
        std::string goal = "0";
        goal[0] += i;
        goal = "leg_left_" + goal + "_link";
        unsigned int id = robotModel.GetBodyId(goal.c_str());
        Eigen::Vector3d position = RigidBodyDynamics::CalcBodyToBaseCoordinates(robotModel, joint_value,
                                                                                id, Eigen::Vector3d::Zero(),
                                                                                true);
        position_set.push_back(position);
    }

    double htar = 0.5;
    double lthigh = position_set[3][2] - position_set[4][2];
    double lshank = position_set[4][2] - position_set[5][2];
    lthigh = 0.2977;
    lshank = 0.2977;
    this->trunk_offset = -1.0*position_set[0][2];

    std::cout<<"trunk offset: "<<trunk_offset<<std::endl;
    double w = 0.3;//2*position_set[0][1];
    double lfoot = 0.22368;
    double wfoot = 0.14651;

    COM_Generation::Set_Robot_Parameters(lthigh, lshank, htar, lfoot, w, wfoot);
}

Three_Mass::~Three_Mass()
{}

void Three_Mass::Set_Mass(double mass_trunk, double mass_leg)
{
    this->Trunk.mass = mass_trunk;
    this->Left.mass = mass_leg;
    this->Right.mass = mass_leg;
    COM_Generation::Set_Mass(mass_trunk+2*mass_leg, mass_leg, mass_trunk, mass_leg, 9.8);
}

void Three_Mass::Visualize()
{
    assert(!(x_left->size[1]^x_right->size[1]^x_trunk->size[1]^y_trunk->size[1]
            ^z_left->size[1]^z_right->size[1]^theta_left->size[1]^theta_right->size[1]));
    int NSample = (int) x_left->size[1];
    for(int i=0; i<NSample; i++)
    {
        base.Set(x_trunk->data[i], y_trunk->data[i], z2+this->trunk_offset, 1, 0, 0, 0);
        Eigen::Quaternion<double> base_ori;
        base_ori.setIdentity();
        UpdateBase(x_trunk->data[i], y_trunk->data[i], z2+this->trunk_offset, base_ori);

        //Three Masses Transformation

        Eigen::Matrix3d ori_left;
        Eigen::Matrix3d ori_right;
        pcl::PointXYZ pos_left;
        pcl::PointXYZ pos_right;
        ori_left.setZero();
        ori_right.setZero();
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
        pos_left.x = x_left->data[i];
        pos_left.y = w/2.0;
        pos_left.z = z_left->data[i];
        pos_right.x = x_right->data[i];
        pos_right.y = -1.0*w/2.0;
        pos_right.z = z_right->data[i];
        element left = element(pos_left, ori_left);
        element right = element(pos_right, ori_right);
        Left.pose = left;
        Right.pose = right;
        Trunk.pose.Set(x_trunk->data[i], y_trunk->data[i], z2, 1, 0, 0, 0);

        if(!Solve(1, Left.pose, Q, true))
            ROS_FATAL_STREAM("Left Leg IK_ERROR on "<<i<<"th iteration");
        else if(!Solve(2, Right.pose, Q, true))
            ROS_FATAL_STREAM("Right Leg IK_ERROR on "<<i<<"th iteration");
        else
        {
            //Publish the result: 1. Broadcast baseTF; 2. Publish Joint Angles
            eMatrixHom baseTf;
            baseTf.setIdentity();
            baseTf(0, 3) = x_trunk->data[i];
            baseTf(1, 3) = y_trunk->data[i];
            baseTf(2, 3) = this->trunk_offset;
            tf::Transform base_tf;
            pal::convert(baseTf, base_tf);
            br.sendTransform(tf::StampedTransform(base_tf, ros::Time::now(), "/world", "/base_link"));
            jointStateMsg.header.stamp = ros::Time::now();
            jointStateMsg.name = jointNames;
            jointStateMsg.position.resize(jointNames.size());
            for(size_t i=0; i<jointNames.size(); i++)
                jointStateMsg.position[i] = Q[i];
            jointPub.publish(jointStateMsg);
        }
        ros::Duration(0.1).sleep();

    }

}

void Three_Mass::Error()
{
    COM_Generation::Error();
}


