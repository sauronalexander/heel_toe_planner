#include <iostream>
#include <ros/ros.h>
#include <heel_toe_planner/COM_Generation/COM_Generation_All.h>
#include <rbdl/rbdl.h>
#include <pal_multicontact_planner/element.h>
#include <pal_multicontact_planner/status.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "heel_toe_planner");
    ros::NodeHandle nh;
    RigidBodyDynamics::Model robotModel;
    std::vector<string> jointNames;
    std::vector<double> position_max;
    std::vector<double> position_min;
    std::vector<double> vel_min;
    std::vector<double> vel_max;
    std::vector<double> damping;
    std::vector<double> friction;
    std::vector<double> max_effort;

    bool floatingBase = false;
    bool planarFloatingBase = false;
    parseUrdfParamServerParameters(robotModel, jointNames, position_min, position_max, vel_min, vel_max,
                                   damping, friction, max_effort, floatingBase, planarFloatingBase);


    Eigen::VectorXd joint_value(jointNames.size());
    joint_value.setZero();
    vector<Eigen::Vector3d> position_set;
    for(int i=1; i<=6; i++)
    {
        string goal = "0";
        goal[0] += i;
        goal = "leg_left_" + goal + "_link";
        unsigned int id = robotModel.GetBodyId(goal.c_str());
        Eigen::Vector3d position = RigidBodyDynamics::CalcBodyToBaseCoordinates(robotModel, joint_value,
                                                                                id, Eigen::Vector3d::Zero(),
                                                                                true);
        position_set.push_back(position);
    }

    double totaltime = 11.5, tinit = 0.7, tend = 0.8, tstep = 1, ratio = 0.1, Period = 0.01;
    double htar = 0.5;
    double lthigh = position_set[3][2] - position_set[4][2];
    double lshank = position_set[4][2] - position_set[5][2];
    lthigh = 0.2977;
    lshank = 0.2977;
    double trunk_offset = position_set[0][2];
    std::cout<<lthigh<<", "<<lshank<<std::endl;
    double w = 0.3;//2*position_set[0][1];
    double lfoot = 0.22368;
    double wfoot = 0.14651;
    double z2 = 0.55, lstep = 0.6;
    COM_Generation COM = COM_Generation();
    COM.Set_Parameters(totaltime, tinit, tend, tstep, ratio, w, wfoot, Period, htar, lfoot, lstep,
                       lthigh, lshank, z2);
    COM.Set_Mass(80.0, 16.0, 48.0, 16.0, 9.8);
    COM.Generate_COM();
    COM.Error();
    return 0;
}
