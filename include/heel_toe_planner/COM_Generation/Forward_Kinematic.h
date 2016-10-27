/*
 * Forward_Kinematic.h
 *
 * Code generation for function 'Forward_Kinematic'
 *
 * C source code generated on: Wed Oct 12 13:19:43 2016
 *
 */

#ifndef __FORWARD_KINEMATIC_H__
#define __FORWARD_KINEMATIC_H__
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
extern void Forward_Kinematic(double theta_hip, double theta_knee, double theta_ankle, double lthigh, double lshank, double lfoot, double *x_cof, double *z_cof, double *x_heel, double *z_heel);
#endif
/* End of code generation (Forward_Kinematic.h) */
