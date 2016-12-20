#include <heel_toe_planner/ZMP_Tracking.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_zmp");
    ros::NodeHandle nh;
    ZMP_Tracking ZMP(nh);
    ZMP.Detect_ZMP();

    return 0;
}
