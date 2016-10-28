#include <heel_toe_planner/IK_Solver.h>

IK_Solver::IK_Solver(ros::NodeHandle & nh)
{
    this->nh = nh;
    std::vector<double> position_max;
    std::vector<double> position_min;
    std::vector<double> vel_min;
    std::vector<double> vel_max;
    std::vector<double> damping;
    std::vector<double> friction;
    std::vector<double> max_effort;
    bool floatingBase = false;
    bool planarFloatingBase = false;

    //Load the Model
    parseUrdfParamServerParameters(robotModel, jointNames, position_min, position_max,
                                                                vel_min, vel_max,
                                                                damping, friction,
                                                                max_effort,
                                                                floatingBase,
                                                                planarFloatingBase);


    limb_names.push_back(" ");
    limb_names.push_back("left_foot");
    limb_names.push_back("right_foot");

    std::string chain_start = "base_link";
    std::string urdf_param = "/robot_description";

    double eps = 1.0e-5;
    double timeout = 0.005;
    leftFootTracIk = new TRAC_IK::TRAC_IK(chain_start, "left_sole_link", urdf_param, timeout, eps);
    rightFootTracIk = new TRAC_IK::TRAC_IK(chain_start, "right_sole_link", urdf_param, timeout, eps);

    KDL::Chain chain;
    KDL::JntArray ll, ul;
    bool valid = leftFootTracIk->getKDLChain(chain);
    if (!valid)
        ROS_ERROR("There was no valid KDL chain found");
    valid = leftFootTracIk->getKDLLimits(ll,ul);
    if (!valid)
        ROS_ERROR("There were no valid KDL joint limits found");
    leftFootNomial.resize(chain.getNrOfJoints());
    for (uint j=0; j<leftFootNomial.data.size(); j++)
    {
        leftFootNomial(j) = (ll(j)+ul(j))/2.0;
    }
    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());

    valid = rightFootTracIk->getKDLChain(chain);
    if (!valid)
        ROS_ERROR("There was no valid KDL chain found");
    valid = rightFootTracIk->getKDLLimits(ll,ul);
    if (!valid)
        ROS_ERROR("There were no valid KDL joint limits found");
    rightFootNomial.resize(chain.getNrOfJoints());
    for (uint j=0; j<rightFootNomial.data.size(); j++)
    {
        rightFootNomial(j) = (ll(j)+ul(j))/2.0;
    }
    assert(chain.getNrOfJoints() == ll.data.size());
    assert(chain.getNrOfJoints() == ul.data.size());
}

eMatrixHom IK_Solver::getFK(const Eigen::VectorXd &state, const Eigen::VectorXd pointOffset,
                           const std::string &linkName, bool updateKinematics)
{

    unsigned body_id = robotModel.GetBodyId(linkName.c_str());

    Eigen::Vector3d position = RigidBodyDynamics::CalcBodyToBaseCoordinates(robotModel, state, body_id, pointOffset, updateKinematics);
    Eigen::Matrix3d rotation = RigidBodyDynamics::CalcBodyWorldOrientation(robotModel, state, body_id, updateKinematics).transpose();

    return createMatrix(rotation, position);
}

IK_Solver::~IK_Solver()
{
    delete leftFootTracIk;
    delete rightFootTracIk;
}

bool IK_Solver::Solve(int limb, element &query, std::vector<double> &Q, bool output_enable)
{
    std::vector<double> Q_prime = Q;
    TRAC_IK::TRAC_IK * tracik_solver;
    KDL::JntArray Nomial;
    switch(limb)
    {
        case 1:
            tracik_solver = leftFootTracIk;
            Nomial = leftFootNomial;
            break;
        case 2:
            tracik_solver = rightFootTracIk;
            Nomial = rightFootNomial;
            break;
        default:
            return false;
            break;

    }



    InteractiveMakerReference baseReference(nh, "base", "world", base_position, base_orientation);
    Eigen::Vector3d target_pos;
    target_pos(0) = query.Get_Pos().x;
    target_pos(1) = query.Get_Pos().y;
    target_pos(2) = query.Get_Pos().z;
    Eigen::Quaternion<double> target_ori(query.Get_Qua()(0), query.Get_Qua()(1), query.Get_Qua()(2), query.Get_Qua()(3));
    this->base_orientation.setIdentity();

    std::vector<std::string> JointNames;
    KDL::Chain chain;
    bool valid = tracik_solver->getKDLChain(chain);
    if (!valid)
    {
        return false;
        ROS_ERROR("There was no valid KDL chain found");
    }
    for(size_t i=0; i<chain.segments.size(); i++)
        JointNames.push_back(chain.segments[i].getJoint().getName());
    for(int i=0; i<JointNames.size(); i++)
        std::cout<<JointNames[i]<<std::endl;

    InteractiveMakerReference targetReference(nh, "base", this->limb_names[limb].c_str(), target_pos, target_ori);
    //Transform baseTf
    eMatrixHom baseTf = baseReference.getPoseTarget();
    eMatrixHom targetTf = targetReference.getPoseTarget();

    KDL::Frame end_effector_pose;
    baseTf = baseTf.inverse();
    double target_basetf[4][4];

    for(int i=0; i<4; i++)
    for(int j=0; j<4; j++)
    {
        target_basetf[i][j] = 0;
        for(int k=0; k<4; k++)
            target_basetf[i][j] += baseTf(i, k)*targetTf(k, j);
        if(i<3)
        {
            if(j==3)
                end_effector_pose.p(i) = target_basetf[i][j];
            else
                end_effector_pose.M(i,j) = target_basetf[i][j];
        }
    }

    KDL::JntArray result;
    int rc = tracik_solver->CartToJnt(Nomial, end_effector_pose, result);
    bool Success;
    if (rc>=0)
        Success = true;
    else
        Success = false;

    if(!Success)
    {
        Q = Q_prime;
        return false;
    }
    // Publish result
    if(output_enable)
        for(size_t i=0; i<JointNames.size(); ++i)
        for(size_t j=0; j<jointNames.size(); ++j)
            if(JointNames[i] == jointNames[j])
            {
                Q[j] = result(i);
                break;
            }
    else
        Q = Q_prime;
    return true;
}


void IK_Solver::UpdateBase(double px, double py, double pz, Eigen::Quaternion<double> orientation)
{
    this->base_position(0) = px;
    this->base_position(1) = py;
    this->base_position(2) = pz;

    this->base_orientation = orientation;
}

Eigen::Matrix3d IK_Solver::GetInvProduct(std::vector<double> configuration, int limb)
{
    Eigen::VectorXd c(jointNames.size());
    for(size_t i=0; i<configuration.size(); i++)
        c(i) = configuration[i];
    unsigned body_id = robotModel.GetBodyId(limb_names[limb].c_str());

    Eigen::Vector3d position = RigidBodyDynamics::CalcBodyToBaseCoordinates(robotModel, c, body_id, Eigen::Vector3d::Zero(), true);
    RigidBodyDynamics::Math::MatrixNd jacobian;
    jacobian.resize(3, jointNames.size());
    jacobian.setOnes();
    RigidBodyDynamics::CalcPointJacobian(robotModel, c, body_id, position, jacobian, true);

    Eigen::Matrix3d t1;
    t1.setZero();
    for(size_t i=0; i<3; i++)
    for(size_t j=0; j<3; j++)
    for(size_t k=0; k<jointNames.size(); k++)
        t1(i,j) += jacobian(i,k) * jacobian(j,k);
    return t1;
}
