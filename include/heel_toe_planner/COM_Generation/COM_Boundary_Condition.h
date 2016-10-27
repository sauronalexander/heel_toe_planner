/*
 * COM_Boundary_Condition.h
 *
 * Code generation for function 'COM_Boundary_Condition'
 *
 * C source code generated on: Wed Oct 12 13:19:43 2016
 *
 */

#ifndef __COM_BOUNDARY_CONDITION_H__
#define __COM_BOUNDARY_CONDITION_H__
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

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void COM_Boundary_Condition(double lthigh, double lshank, double lfoot, double htar, double lstep, double z2, double x_init_pframe[2], double x_init_heel_pframe[2], double x_end_pframe[2], double x_end_heel_pframe[2], double theta_iss[3], double theta_ess[4]);
extern void test_COM_Boundary_Condition();

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (COM_Boundary_Condition.h) */
