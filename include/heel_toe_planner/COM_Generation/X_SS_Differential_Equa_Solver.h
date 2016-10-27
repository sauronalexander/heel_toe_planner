/*
 * X_SS_Differential_Equa_Solver.h
 *
 * Code generation for function 'X_SS_Differential_Equa_Solver'
 *
 * C source code generated on: Wed Oct 19 16:30:45 2016
 *
 */

#ifndef __X_SS_DIFFERENTIAL_EQUA_SOLVER_H__
#define __X_SS_DIFFERENTIAL_EQUA_SOLVER_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"

#include "heel_toe_planner/COM_Generation/rtwtypes.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_types.h"

/* Function Declarations */
extern void X_SS_Differential_Equa_Solver(double g, double h, double A1, double w1, double fai1,
                                          double A3, double w3, double fai3, double Constant,
                                          double x_init, double x_end, double tend, double *A1_prime,
                                          double *A3_prime, double *Constant_prime, double *C1, double *C2);
#endif
/* End of code generation (X_SS_Differential_Equa_Solver.h) */

