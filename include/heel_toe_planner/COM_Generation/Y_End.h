/*
 * Y_End.h
 *
 * Code generation for function 'Y_End'
 *
 * C source code generated on: Wed Oct 12 13:19:43 2016
 *
 */

#ifndef __Y_END_H__
#define __Y_END_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "heel_toe_planner/COM_Generation/rt_defines.h"
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"

#include "heel_toe_planner/COM_Generation/rtwtypes.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_types.h"

/* Function Declarations */
extern void Y_End(double g, double h, double A4, double A3, double A2, double A1, double A0,
                  double Constant, double y_init, double y_end, double tend,
                  double *alpha, double *beta, double *b_gamma, double *fai, double *ita,
                  double *Constant_prime, double *C1, double *C2);
#endif
/* End of code generation (Y_End.h) */
