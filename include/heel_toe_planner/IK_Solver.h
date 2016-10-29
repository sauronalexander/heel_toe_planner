#ifndef IK_H
#define IK_H

#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <cstring>
#include <vector>
#include <rbdl/rbdl.h>
#include <rbdl/addons/rbdlUrdfParser.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
#include <pal_robot_tools/reference/interactive_marker_reference.h>
#include <pal_robot_tools/permutation.h>
#include <pal_robot_tools/conversions.h>
#include <pal_robot_tools/walking_visualization_tools.h>
#include <pal_multicontact_planner/element.h>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <pcl/point_cloud.h>

class IK_Solver
{
    private:
        element base;
        TRAC_IK::TRAC_IK * leftFootTracIk;
        TRAC_IK::TRAC_IK * rightFootTracIk;
        KDL::JntArray leftFootNomial;
        KDL::JntArray rightFootNomial;
        std::vector<std::string> limb_names;
        Eigen::Vector3d base_position;
        Eigen::Quaternion<double> base_orientation;


    protected:
        std::vector<std::string> jointNames;
        ros::NodeHandle nh;
        eMatrixHom getFK(const Eigen::VectorXd &state, const Eigen::VectorXd pointOffset,
                         const std::string &linkName, bool updateKinematics);
        RigidBodyDynamics::Model robotModel;

    public:
        IK_Solver(ros::NodeHandle & nh);
        virtual ~IK_Solver();
        bool Solve(int limb, element &query, std::vector<double> & Q, bool output_enable);
        void UpdateBase(double px, double py, double pz, Eigen::Quaternion<double> orientation);
        Eigen::Matrix3d GetInvProduct(std::vector<double> configuration, int limb);
};

#endif // IK_H
