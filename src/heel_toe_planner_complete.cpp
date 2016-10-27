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
    for(int i=1; i<=6; i++)
    {
        string goal = "0";
        goal[0] += i;
        goal = "leg_left_" + goal + "link";
        unsigned int id = robotModel.GetBodyId(goal.c_str());
        Eigen::Vector3d position = RigidBodyDynamics::CalcBodyToBaseCoordinates(robotModel, joint_value,
                                                                                id, Eigen::Vector3d::Zero(),
                                                                                true);
        std::cout<<"link "<<i<<" : "<<position<<std::endl;
    }
    COM_Generation COM = COM_Generation();
    COM.Set_Parameters(11.5, 0.7, 0.8, 1.0, 0.1, 0.4, 0.1, 0.01, 0.2, 0.2, 0.5, 0.35, 0.35, 0.69);
    COM.Set_Mass(80.0, 16.0, 48.0, 16.0, 9.8);
    COM.Generate_COM();
    COM.Error();
    return 0;
}
