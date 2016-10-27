/*
 * Heel_Toe_Planner_Part1_emxAPI.h
 *
 * Code generation for function 'Heel_Toe_Planner_Part1_emxAPI'
 *
 * C source code generated on: Wed Oct 12 13:19:44 2016
 *
 */

#ifndef __HEEL_TOE_PLANNER_PART1_EMXAPI_H__
#define __HEEL_TOE_PLANNER_PART1_EMXAPI_H__
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
extern emxArray_real_T *emxCreateND_real_T(int numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapperND_real_T(double *data, int numDimensions, int *size);
extern emxArray_real_T *emxCreateWrapper_real_T(double *data, int rows, int cols);
extern emxArray_real_T *emxCreate_real_T(int rows, int cols);
extern void emxDestroyArray_real_T(emxArray_real_T *emxArray);
#endif
/* End of code generation (Heel_Toe_Planner_Part1_emxAPI.h) */
