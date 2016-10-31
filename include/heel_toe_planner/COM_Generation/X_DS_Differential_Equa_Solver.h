/*
 * X_DS_Differential_Equa_Solver.h
 *
 * Code generation for function 'X_DS_Differential_Equa_Solver'
 *
 * C source code generated on: Wed Oct 19 16:30:45 2016
 *
 */

#ifndef __X_DS_DIFFERENTIAL_EQUA_SOLVER_H__
#define __X_DS_DIFFERENTIAL_EQUA_SOLVER_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"

#include "heel_toe_planner/COM_Generation/rtwtypes.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_types.h"

/* Function Declarations */
extern void X_DS_Differential_Equa_Solver(double Start, double End, emxArray_real_T * t, double g, double h, double lfoot, double tds, const double theta_ess[4], const double theta_iss[3], double m1, double m2, double m3, double ze, double A2, double w2, double fai2, double O2, double t0, double x1_0, double x3_0, double x2_init, double x2_end, emxArray_real_T *x1, emxArray_real_T *x2, emxArray_real_T *x3, emxArray_real_T *z1, emxArray_real_T *z3, emxArray_real_T *theta_1, emxArray_real_T *theta_3);
#endif
/* End of code generation (X_DS_Differential_Equa_Solver.h) */
