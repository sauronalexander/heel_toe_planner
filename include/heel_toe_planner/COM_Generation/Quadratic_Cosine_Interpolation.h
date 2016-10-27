/*
 * Quadratic_Cosine_Interpolation.h
 *
 * Code generation for function 'Quadratic_Cosine_Interpolation'
 *
 * C source code generated on: Wed Oct 12 13:19:43 2016
 *
 */

#ifndef __QUADRATIC_COSINE_INTERPOLATION_H__
#define __QUADRATIC_COSINE_INTERPOLATION_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "heel_toe_planner/COM_Generation/rt_defines.h"
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"

#include "heel_toe_planner/COM_Generation/rtwtypes.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_types.h"
#include "heel_toe_planner/COM_Generation/Ipopt_Solver/NLP_Interface.hpp"

/* Function Declarations */
extern void Quadratic_Cosine_Interpolation(double delta01, double delta12, double delta23, double y_zmp0, double y_zmp1, double y_zmp3, double y_zmp_prime0, double coef_y[6]);
#endif
/* End of code generation (Quadratic_Cosine_Interpolation.h) */
