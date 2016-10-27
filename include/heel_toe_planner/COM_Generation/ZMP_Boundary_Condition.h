/*
 * ZMP_Boundary_Condition.h
 *
 * Code generation for function 'ZMP_Boundary_Condition'
 *
 * C source code generated on: Wed Oct 12 13:19:44 2016
 *
 */

#ifndef __ZMP_BOUNDARY_CONDITION_H__
#define __ZMP_BOUNDARY_CONDITION_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "heel_toe_planner/COM_Generation/rt_defines.h"
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"

#include "heel_toe_planner/COM_Generation/rtwtypes.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void ZMP_Boundary_Condition(double totaltime, double tinit, double tend, double tstep, double ratio, double Period, double unusedU0, double lfoot, double w,double wfoot,
                                   emxArray_real_T * leftgait_x, emxArray_real_T * rightgait_x, emxArray_real_T * leftgait_y, emxArray_real_T * rightgait_y,
                                   emxArray_real_T * zmpUB_x, emxArray_real_T * zmpLB_x,
                                   emxArray_real_T * zmpUB_y, emxArray_real_T * zmpLB_y,
                                   emxArray_real_T * zmpUB_x_t, emxArray_real_T * zmp_LB_x_t);
extern void test_zmp_boundary();

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (ZMP_Boundary_Condition.h) */
