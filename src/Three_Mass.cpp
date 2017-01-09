#include <heel_toe_planner/Three_Mass.h>

Three_Mass::Three_Mass(ros::NodeHandle & nh) : IK_Solver(nh), COM_Generation()
{
    //Initialize Publisher
    jointPub = nh.advertise<sensor_msgs::JointState>("/joint_states", 100);
    //Initialize Joint State Message
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
    lthigh = 0.2993;//0.2977;
    lshank = 0.2993;//0.2977;
    this->trunk_offset = -1.0*position_set[0][2];

    std::cout<<"Trunk offset: "<<trunk_offset<<std::endl;
    double w = 0.23;//2*position_set[0][1];
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

void Three_Mass::Generate_Reference_Angles(bool visualize)
{
    ROS_INFO("Start Generation of Reference Angles...");
    if(visualize)
        ROS_INFO("Start Visualization in RVIZ...");
    assert(!(x_left->size[1]^x_right->size[1]^x_trunk->size[1]^y_trunk->size[1]
            ^z_left->size[1]^z_right->size[1]^theta_left->size[1]^theta_right->size[1]));

    int NSample = (int) x_left->size[1];
    for(int i=0; i<NSample; i++)
    {
        base.Set(x_trunk->data[i], -1.0*y_trunk->data[i], z2+this->trunk_offset, 1, 0, 0, 0);
        Eigen::Quaternion<double> base_ori;
        base_ori.setIdentity();
        UpdateBase(x_trunk->data[i], -1.0*y_trunk->data[i], z2+this->trunk_offset, base_ori);

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
        pos_right.z = z_right->data[i];
        element left = element(pos_left, ori_left);
        element right = element(pos_right, ori_right);
        Left.pose = left;
        Right.pose = right;
        Trunk.pose.Set(x_trunk->data[i], -1.0*y_trunk->data[i], z2, 1, 0, 0, 0);

        bool left_solve = false;
        bool right_solve = false;

        for(int k=0; k<3; k++)
            if(Solve(1, Left.pose, Q, true))
            {
                left_solve = true;
                break;
            }
        for(int k=0; k<3; k++)
            if(Solve(2, Right.pose, Q, true))
            {
                right_solve = true;
                break;
            }

        if(!left_solve)
        {
            error_vector.push_back(i);
            ROS_FATAL_STREAM("Left Leg IK_ERROR on "<<i<<"th iteration");
        }
        else if(!right_solve)
        {
            error_vector.push_back(i);
            ROS_FATAL_STREAM("Right Leg IK_ERROR on "<<i<<"th iteration");
        }
        else
        {
            //Publish the result: 1. Broadcast baseTF; 2. Publish Joint Angles
            eMatrixHom baseTf;
            baseTf.setIdentity();
            baseTf(0, 3) = x_trunk->data[i];
            baseTf(1, 3) = -1.0*y_trunk->data[i];
            baseTf(2, 3) = this->trunk_offset;
            tf::Transform base_tf;
            pal::convert(baseTf, base_tf);
            if(visualize)
                br.sendTransform(tf::StampedTransform(base_tf, ros::Time::now(), "/world", "/base_link"));
            jointStateMsg.header.stamp = ros::Time::now();
            jointStateMsg.name = jointNames;
            jointStateMsg.position.resize(jointNames.size());
            for(size_t i=0; i<jointNames.size(); i++)
                jointStateMsg.position[i] = Q[i];
            this->Reference_Angles.push_back(Q);
            if(visualize)
                jointPub.publish(jointStateMsg);
        }
        if(i%(NSample/10) == 0)
            ROS_INFO_STREAM(10*i/(NSample/10)<<" percent completed!");
        if(visualize)
            ros::Duration(0.1).sleep();

    }

}

void Three_Mass::Error()
{
    COM_Generation::Error();
    ROS_WARN_STREAM(error_vector.size()<<" invalid IK configurations");
    ROS_WARN_STREAM("IK Fail Rate: "<<((double) error_vector.size()/(double)x_left->size[1]) * 100.0<<"%");
}

void Three_Mass::read()
{
    std::string path = ros::package::getPath("heel_toe_planner");
    std::string temp_path;

    ROS_INFO_STREAM("Begin Fetching Of Reference Angles...");
    temp_path = path + "/database/Reference_Angles.bin";
    ROS_INFO_STREAM("From File: "<<temp_path);
    std::ifstream ifs1(temp_path.c_str());
    boost::archive::binary_iarchive ia1(ifs1);
    int num_joint_angles;
    ia1 >> num_joint_angles;
    ia1 >> this->Standing_Pose;
    for(int i=0; i<num_joint_angles; i++)
    {
        std::vector<double> joint_config;
        ia1 >> joint_config;
        this->Reference_Angles.push_back(joint_config);
    }
    ifs1.close();
    ROS_INFO_STREAM("Fetching of Reference Angles Completed...");


    ROS_INFO_STREAM("Begin Fetching Of Reference ZMP Trajectories...");
    temp_path = path + "/database/ZMP_Trajectories.bin";
    ROS_INFO_STREAM("From File: "<<temp_path);
    std::ifstream ifs2(temp_path.c_str());
    boost::archive::binary_iarchive ia2(ifs2);
    int size;
    int size_array;

    ia2 >> size;
    emxInit_real_T(&this->zmp_x, 2);
    size_array = this->zmp_x->size[0] * this->zmp_x->size[1];
    this->zmp_x->size[0] = 1;
    this->zmp_x->size[1] = size;
    emxEnsureCapacity((emxArray__common *) zmp_x, size_array, (int)sizeof(double));
    for(int i=0; i<this->zmp_x->size[1]; i++)
        ia2 >> this->zmp_x->data[i];

    ia2 >> size;
    emxInit_real_T(&this->zmp_y, 2);
    size_array = this->zmp_y->size[0] * this->zmp_y->size[1];
    this->zmp_y->size[0] = 1;
    this->zmp_y->size[1] = size;
    emxEnsureCapacity((emxArray__common *) zmp_y, size_array, (int)sizeof(double));
    for(int i=0; i<this->zmp_y->size[1]; i++)
        ia2 >> this->zmp_y->data[i];
    ifs2.close();
    ROS_INFO_STREAM("Fecthing of Reference ZMP Trajectories Completed...");
}

void Three_Mass::write()
{
    assert(this->Reference_Angles.size());
    if(!this->Standing_Pose.size())
        Solve_Standing_Pose();
    assert(this->Standing_Pose.size());
    std::string path = ros::package::getPath("heel_toe_planner");
    std::string temp_path;

    ROS_INFO_STREAM("Begin Serialization Of Reference Angles...");
    temp_path = path + "/database/Reference_Angles.bin";
    ROS_INFO_STREAM("Saving To File: "<<temp_path);
    std::ofstream ofs1(temp_path.c_str());
    boost::archive::binary_oarchive oa1(ofs1);
    int size = (int) this->Reference_Angles.size();
    oa1 << size;
    oa1 << this->Standing_Pose;
    for(size_t i=0; i<Reference_Angles.size(); i++)
        oa1 << this->Reference_Angles[i];
    ofs1.close();
    ROS_INFO_STREAM("Serialization of Reference Angles Completed...");


    ROS_INFO_STREAM("Begin Serialization Of Reference ZMP Trajectories...");
    temp_path = path + "/database/ZMP_Trajectories.bin";
    ROS_INFO_STREAM("Saving To File: "<<temp_path);
    std::ofstream ofs2(temp_path.c_str());
    boost::archive::binary_oarchive oa2(ofs2);
    oa2 << this->zmp_x->size[1];
    for(int i=0; i<this->zmp_x->size[1]; i++)
        oa2 << this->zmp_x->data[i];
    oa2 << this->zmp_y->size[1];
    for(int i=0; i<this->zmp_y->size[1]; i++)
        oa2 << this->zmp_y->data[i];
    ofs2.close();
    ROS_INFO_STREAM("Serialization of Reference ZMP Trajectories Completed...");

    if(Verify())
        ROS_INFO_STREAM("Serialization Complete");
    else
        ROS_ERROR_STREAM("Serialization Failed");

}

bool Three_Mass::Verify()
{
    std::string path = ros::package::getPath("heel_toe_planner");
    std::string temp_path;

    ROS_INFO_STREAM("Begin Fetching Of Reference Angles...");
    temp_path = path + "/database/Reference_Angles.bin";
    ROS_INFO_STREAM("From File: "<<temp_path);
    std::ifstream ifs1(temp_path.c_str());
    boost::archive::binary_iarchive ia1(ifs1);
    int num_joint_angles;
    ia1 >> num_joint_angles;
    std::vector<double> stand;
    ia1 >> stand;
    if(this->Standing_Pose.size() != stand.size())
        return false;
    for(size_t i =0; i<this->Standing_Pose.size(); i++)
        if(this->Standing_Pose[i] != stand[i])
            return false;
    if(num_joint_angles != this->Reference_Angles.size())
        return false;
    std::vector<std::vector<double> > Reference_Angles_prime;
    for(int i=0; i<num_joint_angles; i++)
    {
        std::vector<double> joint_config;
        ia1 >> joint_config;
        Reference_Angles_prime.push_back(joint_config);
    }
    for(size_t i=0; i<Reference_Angles_prime.size(); i++)
    {
        if(Reference_Angles_prime[i].size() != this->Reference_Angles[i].size())
            return false;
        for(size_t j=0; j<Reference_Angles_prime[j].size(); j++)
            if(Reference_Angles_prime[i][j] != this->Reference_Angles[i][j])
                return false;
    }
    ifs1.close();
    ROS_INFO_STREAM("Fetching of Reference Angles Completed...");


    ROS_INFO_STREAM("Begin Fetching Of Reference ZMP Trajectories...");
    temp_path = path + "/database/ZMP_Trajectories.bin";
    ROS_INFO_STREAM("From File: "<<temp_path);
    bool flag = true;
    std::ifstream ifs2(temp_path.c_str());
    boost::archive::binary_iarchive ia2(ifs2);
    int size;
    int size_array;

    emxArray_real_T* zmp_x_prime;
    emxArray_real_T* zmp_y_prime;
    zmp_x_prime = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    zmp_y_prime = (emxArray_real_T*) malloc(sizeof(emxArray_real_T));
    Initial_emxArry(zmp_x_prime);
    Initial_emxArry(zmp_y_prime);

    ia2 >> size;
    emxInit_real_T(&zmp_x_prime, 2);
    size_array = zmp_x_prime->size[0] * zmp_x_prime->size[1];
    zmp_x_prime->size[0] = 1;
    zmp_x_prime->size[1] = size;
    if(zmp_x->size[1] != size)
        flag = false;
    emxEnsureCapacity((emxArray__common *) zmp_x_prime, size_array, (int)sizeof(double));
    for(int i=0; i<zmp_x_prime->size[1]; i++)
    {
        ia2 >> zmp_x_prime->data[i];
        if(zmp_x_prime->data[i] != this->zmp_x->data[i])
            flag = false;
    }

    ia2 >> size;
    emxInit_real_T(&zmp_y_prime, 2);
    size_array = zmp_y_prime->size[0] * zmp_y_prime->size[1];
    zmp_y_prime->size[0] = 1;
    zmp_y_prime->size[1] = size;
    emxEnsureCapacity((emxArray__common *) zmp_y_prime, size_array, (int)sizeof(double));
    for(int i=0; i<zmp_y_prime->size[1]; i++)
    {
        ia2 >> zmp_y_prime->data[i];
        if(zmp_y_prime->data[i] != this->zmp_y->data[i])
            flag = false;
    }
    ROS_INFO_STREAM("Fecthing of Reference ZMP Trajectories Completed...");

    Delete_emxArry(zmp_x_prime);
    Delete_emxArry(zmp_y_prime);
    zmp_x_prime = NULL;
    zmp_y_prime = NULL;
    ifs2.close();
    return flag;
}

void Three_Mass::Solve_Standing_Pose()
{
    ROS_INFO("Start Generating Default Standing Pose...");
    base.Set(x_trunk->data[0], 0, z2+this->trunk_offset, 1, 0, 0, 0);
    Eigen::Quaternion<double> base_ori;
    base_ori.setIdentity();
    UpdateBase(x_trunk->data[0], 0, z2+this->trunk_offset, base_ori);

    //Solve For Standing Pose
    this->Standing_Pose.resize(this->jointNames.size());
    Eigen::Matrix3d ori_left;
    Eigen::Matrix3d ori_right;
    pcl::PointXYZ pos_left;
    pcl::PointXYZ pos_right;
    ori_left.setZero();
    ori_right.setZero();

    pos_left.x = x_left->data[0];
    pos_left.y = w/2.0;
    pos_left.z = 0.0;
    pos_right.x = x_right->data[0];
    pos_right.y = -1.0*w/2.0;
    pos_right.z = z_right->data[0];

    ori_right(0, 0) = cos(theta_right->data[0]);
    ori_right(0, 2) = sin(theta_right->data[0]);
    ori_right(1, 1) = 1.0;
    ori_right(2, 0) = -1.0*sin(theta_right->data[0]);
    ori_right(2, 2) = cos(theta_right->data[0]);

    ori_left = ori_right;

    element left = element(pos_left, ori_left);
    element right = element(pos_right, ori_right);

    Left.pose = left;
    Right.pose = right;
    //Trunk.pose.Set(x_trunk->data[0], -1.0*y_trunk->data[0], z2, 1, 0, 0, 0);

    bool left_solve = false;
    bool right_solve = false;

    for(int k=0; k<3; k++)
        if(Solve(1, Left.pose, this->Standing_Pose, true))
        {
            left_solve = true;
            break;
        }
    for(int k=0; k<3; k++)
        if(Solve(2, Right.pose, this->Standing_Pose, true))
        {
            right_solve = true;
            break;
        }
    if(!right_solve)
        ROS_ERROR("Right Foot Standing Pose Error!");
    if(!left_solve)
        ROS_ERROR("Left Foot Standing Pose Error!");

//    for(size_t i=0; i<this->Standing_Pose.size();i++)
//        std::cout<<Standing_Pose[i]<<std::endl;
}
