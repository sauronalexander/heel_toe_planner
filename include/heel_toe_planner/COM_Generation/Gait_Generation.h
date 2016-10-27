/*
 * Gait_Generation.h
 *
 * Code generation for function 'Gait_Generation'
 *
 * C source code generated on: Wed Oct 12 13:19:43 2016
 *
 */

#ifndef __GAIT_GENERATION_H__
#define __GAIT_GENERATION_H__
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

#ifdef __cplusplus
extern "C" {
#endif
extern void Gait_Generation(double totaltime, double tinit, double tend, double tstep, double ratio, double Period, double lstep, double unusedU0, double w, double unusedU1, emxArray_real_T *leftgait_x, emxArray_real_T *rightgait_x, emxArray_real_T *leftgait_y, emxArray_real_T *rightgait_y);
extern void test_gait();
#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (Gait_Generation.h) */
