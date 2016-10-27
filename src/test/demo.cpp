#include <ros/ros.h>
#include <rbdl/rbdl.h>
#include <pal_multicontact_planner/element.h>


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "demo");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("DEMO");
    element temp();

    return 0;
}
