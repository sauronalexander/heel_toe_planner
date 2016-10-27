#include <iostream>
#include <ros/ros.h>
#include <heel_toe_planner/COM_Generation/COM_Generation_All.h>
#include <rbdl/rbdl.h>
#include <pal_multicontact_planner/element.h>
#include <pal_multicontact_planner/status.h>

int main(int argc, char** argv)
{
    COM_Generation COM = COM_Generation();
    COM.Set_Parameters(11.5, 0.7, 0.8, 1.0, 0.1, 0.4, 0.1, 0.01, 0.2, 0.2, 0.5, 0.35, 0.35, 0.69);
    COM.Set_Mass(80.0, 16.0, 48.0, 16.0, 9.8);
    COM.Generate_COM();
    COM.Error();
    return 0;
}
