#ifndef NLP_INTERFACE_H
#define NLP_INTERFACE_H

#include <stdbool.h>

#ifdef __cplusplus
#include <iostream>
#include <ros/ros.h>
#include <heel_toe_planner/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver1.hpp>
#include <heel_toe_planner/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver2.hpp>
#include <heel_toe_planner/COM_Generation/Ipopt_Solver/Nonlinear_Eqn_Solver3.hpp>
#include <coin/IpTNLP.hpp>
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern double NLP1_Solver(double yzmp_1, double delta13, double yzmp_1_prime, double yzmp_3);
extern bool NLP2_Solver(double lthigh, double lshank, double lfoot, double lstep, double z2,
                          double htar, double * theta_i);
extern bool NLP3_Solver(double lthigh, double lshank, double lfoot, double lstep, double z2,
                          double htar, double zi, double * theta_e);

#ifdef __cplusplus
}
#endif

#endif // NLP_INTERFACE_H
