#include <iostream>
#include <ros/ros.h>
#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
#include <cassert>
#include <cmath>
#include <heel_toe_planner/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver1.hpp>
#include <heel_toe_planner/COM_Generation/Ipopt_Solver/NLP_Interface.hpp>
using namespace::Ipopt;
int main(int argv, char** argc)
{
//    Ipopt::SmartPtr<Ipopt::TNLP> mynlp = new NLP1(0.15, 0.1, -0.1429, 0.15);
//    Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
//    app->Options()->SetNumericValue("tol", 1e-7);
//    app->Options()->SetStringValue("mu_strategy", "adaptive");
//    app->Options()->SetStringValue("output_file", "ipopt.out");
//    app->Options()->SetIntegerValue("print_level", 0);

//    Ipopt::ApplicationReturnStatus status;
//    status = app->Initialize();
//    if(status != Ipopt::Solve_Succeeded)
//    {
//        ROS_ERROR_STREAM("*** Error during initialization!");
//        return (int) status;
//    }
//    status = app->OptimizeTNLP(mynlp);
//    if (status == Ipopt::Solve_Succeeded) {
//        // Retrieve some statistics about the solve
//        Ipopt::Index iter_count = app->Statistics()->IterationCount();
//        std::cout << std::endl << std::endl << "*** The problem solved in " << iter_count << " iterations!" << std::endl;

//        Ipopt::Number final_obj = app->Statistics()->FinalObjective();
//        std::cout << std::endl << std::endl << "*** The final value of the objective function is " << final_obj << '.' << std::endl;
//    }

//    return (int) status;
    double w3 = NLP1_Solver(0.076745, 0.1, -0.2093, -0.07674570);
    std::cout<<w3<<std::endl;
    w3 = NLP1_Solver(0.076745, 0.1, -0.35493395, -0.07674570);
    std::cout<<w3<<std::endl;
    double * theta_i = (double*) malloc(3*sizeof(double));
    NLP2_Solver(0.2977, 0.2977, 0.2237, 0.6, 0.55, 0.5, theta_i);
    for(int i=0; i<3; i++)
        std::cout<<theta_i[i]<<", ";
    std::cout<<std::endl;
    double * theta_e = (double*) malloc(4*sizeof(double));
    NLP3_Solver(0.2977, 0.2977, 0.2237, 0.6, 0.55, 0.5, 0.0910, theta_e);
    for(int i=0; i<4; i++)
        std::cout<<theta_e[i]<<", ";
    std::cout<<std::endl;
    delete theta_i;
    delete theta_e;
    return 0;
}
