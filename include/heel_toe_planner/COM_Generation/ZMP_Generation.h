/*
 * ZMP_Generation.h
 *
 * Code generation for function 'ZMP_Generation'
 *
 * C source code generated on: Wed Oct 12 13:19:44 2016
 *
 */

#ifndef __ZMP_GENERATION_H__
#define __ZMP_GENERATION_H__
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
extern void ZMP_Generation(emxArray_real_T * zmpLB_x_t, emxArray_real_T * zmpUB_x_t, double totaltime,
                           double tinit, double tend, double tstep, double ratio, double Period,
                           double lstep, double lfoot, double w, double wfoot, double htar,
                           emxArray_real_T *zmp_x, emxArray_real_T *zmp_y, emxArray_real_T *delta_y,
                           emxArray_real_T *coef_x, emxArray_real_T *coef_y);
extern void test_ZMP_Generation();

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (ZMP_Generation.h) */
