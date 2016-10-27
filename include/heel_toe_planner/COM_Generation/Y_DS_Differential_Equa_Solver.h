/*
 * Y_DS_Differential_Equa_Solver.h
 *
 * Code generation for function 'Y_DS_Differential_Equa_Solver'
 *
 * C source code generated on: Wed Oct 19 16:30:45 2016
 *
 */

#ifndef __Y_DS_DIFFERENTIAL_EQUA_SOLVER_H__
#define __Y_DS_DIFFERENTIAL_EQUA_SOLVER_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"

#include "heel_toe_planner/COM_Generation/rtwtypes.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_types.h"

/* Function Declarations */
extern void Y_DS_Differential_Equa_Solver(double Start, double End, const double t_data[1151], const int t_size[2], double g, double h, double lfoot, double tds, const double theta_ess[4], const double theta_iss[3], double m1, double m2, double m3, double b_y1, double y3, double B, double wb, double faib, double t0, double y2_init, double y2_end, emxArray_real_T *y2);
extern void b_Y_DS_Differential_Equa_Solver(double Start, double End, const emxArray_real_T *t, double g, double h, double lfoot, double tds, const double theta_ess[4], const double theta_iss[3], double m1, double m2, double m3, double b_y1, double y3, double B, double wb, double faib, double t0, double y2_init, double y2_end, emxArray_real_T *y2);
#endif
/* End of code generation (Y_DS_Differential_Equa_Solver.h) */
