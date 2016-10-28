#ifndef THREE_MASS_H
#define THREE_MASS_H

#include <heel_toe_planner/IK_Solver.h>
#include <pal_multicontact_planner/element.h>
#include <heel_toe_planner/COM_Generation/COM_Generation_All.h>
#include <tf/transform_broadcaster.h>
#include <cassert>
#include <sensor_msgs/JointState.h>
#include <pcl/point_cloud.h>



typedef struct Mass_Info
{
    element pose;
    double mass;
} Mass_Info;

class Three_Mass : public IK_Solver, public COM_Generation
{
    private:
        Mass_Info Trunk;
        Mass_Info Left;
        Mass_Info Right;
        element base;
        double trunk_offset;

    protected:
        ros::Publisher jointPub;
        tf::TransformBroadcaster br;
        sensor_msgs::JointState jointStateMsg;
        std::vector<double> Q;

    public:
        Three_Mass(ros::NodeHandle & nh);
        void Set_Mass(double mass_trunk, double mass_leg);
        void Visualize();
        ~Three_Mass();
        void Error();

};


#endif // THREE_MASS_H