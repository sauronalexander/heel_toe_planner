/*
 * Heel_Toe_Planner_Part1_emxutil.h
 *
 * Code generation for function 'Heel_Toe_Planner_Part1_emxutil'
 *
 * C source code generated on: Wed Oct 12 13:19:44 2016
 *
 */

#ifndef __HEEL_TOE_PLANNER_PART1_EMXUTIL_H__
#define __HEEL_TOE_PLANNER_PART1_EMXUTIL_H__
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
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int elementSize);
extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_real_T(emxArray_real_T **pEmxArray);
extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions);
#endif
/* End of code generation (Heel_Toe_Planner_Part1_emxutil.h) */
