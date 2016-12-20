#include <heel_toe_planner/ZMP_Callback.h>

ZMP_Callback::ZMP_Callback()
{
    left_count = 0;
    right_count = 0;
    states_count = 0;
    link_state_time = ros::Time::now();
}

void ZMP_Callback::Receive_Wrench(const geometry_msgs::WrenchStamped &msg_left, const geometry_msgs::WrenchStamped &msg_right)
{
    //Needs to transform into base frame (Left)
    ros::Time time_error = msg_left.header.stamp.now();
    left_sensor = msg_left.header.stamp.now();
    ROS_INFO_STREAM("Time Error for Left Leg: "<<time_error.toNSec());
    Eigen::Matrix3d transform = this->left6link.Get_Inverse_Porduct();
    Eigen::Vector3d force(msg_left.wrench.force.x, msg_left.wrench.force.y, msg_left.wrench.force.z);
    Eigen::Vector3d torque(msg_left.wrench.torque.x, msg_left.wrench.torque.y, msg_left.wrench.torque.z);

    Eigen::Vector3d iforce = transform * force;
    Eigen::Vector3d itorque = transform * torque;

    this->LeftLeg.time = msg_left.header.stamp.now();
    this->LeftLeg.force = iforce;
    this->LeftLeg.torque = itorque;


    //Needs to transform into base frame
    time_error = msg_right.header.stamp.now();
    left_sensor = msg_right.header.stamp.now();
    ROS_INFO_STREAM("Time Error for Right Leg: "<<time_error.toNSec());
    transform = this->left6link.Get_Inverse_Porduct();
    force = Eigen::Vector3d(msg_right.wrench.force.x, msg_right.wrench.force.y, msg_right.wrench.force.z);
    torque = Eigen::Vector3d(msg_right.wrench.torque.x, msg_right.wrench.torque.y, msg_right.wrench.torque.z);

    iforce = transform * force;
    itorque = transform * torque;

    this->RightLeg.time = msg_right.header.stamp.now();
    this->RightLeg.force = iforce;
    this->RightLeg.torque = itorque;
}

void ZMP_Callback::Receive_Wrench_left(const geometry_msgs::WrenchStamped &msg)
{
    //Needs to transform into base frame
    uint64_t time_error = msg.header.stamp.now().toNSec() - this->link_state_time.toNSec();
    left_sensor = msg.header.stamp.now();
    ROS_INFO_STREAM("Time Error for Left Leg: "<<time_error);
    Eigen::Matrix3d transform = this->left6link.Get_Inverse_Porduct();
    Eigen::Vector3d force(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    Eigen::Vector3d torque(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);

    Eigen::Vector3d iforce = transform * force;
    Eigen::Vector3d itorque = transform * torque;

    this->LeftLeg.time = msg.header.stamp.now();
    this->LeftLeg.force = iforce;
    this->LeftLeg.torque = itorque;

}

void ZMP_Callback::Receive_Wrench_right(const geometry_msgs::WrenchStamped &msg)
{
    //Needs to transform into base frame
    uint64_t time_error = msg.header.stamp.now().toNSec() - this->link_state_time.toNSec();
    ROS_INFO_STREAM("Time Error for Left Leg: "<<time_error);
    Eigen::Matrix3d transform = this->right6link.Get_Inverse_Porduct();
    Eigen::Vector3d force(msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z);
    Eigen::Vector3d torque(msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z);

    Eigen::Vector3d iforce = transform * force;
    Eigen::Vector3d itorque = transform * torque;

    this->RightLeg.time = msg.header.stamp.now();
    this->RightLeg.force = iforce;
    this->RightLeg.torque = itorque;
}

void ZMP_Callback::Receive_Link_States(const gazebo_msgs::LinkStates &msg)
{
    link_state_time = ros::Time::now();
    for(size_t i=0; i<msg.name.size(); i++)
    {
        if(msg.name[i] == "reemc_full::leg_left_6_link")
            this->left6link.Set(msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z,
                                msg.pose[i].orientation.w, msg.pose[i].orientation.x, msg.pose[i].orientation.y,
                                msg.pose[i].orientation.z);
        else if(msg.name[i] == "reemc_full::leg_right_6_link")
            this->right6link.Set(msg.pose[i].position.x, msg.pose[i].position.y, msg.pose[i].position.z,
                                 msg.pose[i].orientation.w, msg.pose[i].orientation.x, msg.pose[i].orientation.y,
                                 msg.pose[i].orientation.z);
    }
}
