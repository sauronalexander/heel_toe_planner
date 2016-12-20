#include <heel_toe_planner/ZMP_Tracking.h>

ZMP_Tracking::ZMP_Tracking(ros::NodeHandle &node_handle)
{
    nh = node_handle;
    callback = ZMP_Callback();

//    subLeft = message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh, "/left_ft", 2000);
//    subRight = message_filters::Subscriber<geometry_msgs::WrenchStamped>(nh, "/right_ft", 2000);
//    sync = message_filters::Synchronizer<FTSSyncPolicy>(FTSSyncPolicy(2000), subLeft, subRight);
//    sync.registerCallback(boost::bind(&callback.Receive_Wrench, _1, _2));

}



void ZMP_Tracking::ZMP_Calculation()
{
    uint64_t error = callback.LeftLeg.time.toNSec() - callback.RightLeg.time.toNSec();
    if(abs(error) < 1.0)
        ROS_ERROR_STREAM("Left Right Sync Error Exceeds Limit: "<<error);

    double h_right = callback.right6link.Get_Pos().z;
    double h_left = callback.left6link.Get_Pos().z;

    this->ZMP_x = (callback.RightLeg.torque(1)+callback.LeftLeg.torque(1)+callback.RightLeg.force(0)*h_right
                   +callback.LeftLeg.force(0)*h_left) / (callback.LeftLeg.force(2)+callback.RightLeg.force(2));

    this->ZMP_y = -(callback.RightLeg.torque(0)+callback.LeftLeg.torque(0)+callback.RightLeg.force(1)*h_right
                   +callback.LeftLeg.force(1)*h_left) / (callback.LeftLeg.force(2)+callback.RightLeg.force(2));

    ROS_INFO_STREAM("ZMP Coordinates: "<<this->ZMP_x<<", "<<this->ZMP_y);
    //Which Frame?
}

void ZMP_Tracking::Detect_ZMP()
{
    ROS_INFO("Detecting ZMP...");
    heel_toe_planner::ZMPCoordinate msg;

    ros::Subscriber sub_left = nh.subscribe("/left_ft", 2000, &ZMP_Callback::Receive_Wrench_left, &callback);
    ros::Subscriber sub_right = nh.subscribe("/right_ft", 2000, &ZMP_Callback::Receive_Wrench_right, &callback);
    ros::Subscriber sub_states = nh.subscribe("/gazebo/link_states", 2000, &ZMP_Callback::Receive_Link_States, &callback);
    pub_ZMP = nh.advertise<heel_toe_planner::ZMPCoordinate>("/ZMP_Coordinate", 2000);

    while(ros::ok())
    {
        //ZMP_Calculation();
        msg.x = this->ZMP_x;
        msg.y = this->ZMP_y;
        //pub_ZMP.publish(msg);
    }
}
