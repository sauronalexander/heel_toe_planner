#include <iostream>
#include <ros/ros.h>
#include <heel_toe_planner/COM_Generation/COM_Generation_All.h>
#include <rbdl/rbdl.h>
#include <rbdl/addons/rbdlUrdfParser.h>
#include <pal_multicontact_planner/element.h>
#include <pal_multicontact_planner/status.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <heel_toe_planner/Three_Mass.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "heel_toe_planner");
    ros::NodeHandle nh;
    double totaltime = 11.5, tinit = 0.7, tend = 0.8, tstep = 1, ratio = 0.1, Period = 0.01;
    double z2 = 0.55, lstep = 0.6;
    Three_Mass ThreeMassModel = Three_Mass(nh);
    ThreeMassModel.Set_Parameters(totaltime, tinit, tend, tstep, ratio, Period, lstep, z2);
    ThreeMassModel.Set_Mass(48.0, 16.0);
    ThreeMassModel.Generate_COM();
    ThreeMassModel.Visualize();
    ThreeMassModel.Error();
    return 0;
}
