/*
 * Forward_Kinematic.c
 *
 * Code generation for function 'Forward_Kinematic'
 *
 * C source code generated on: Wed Oct 12 13:19:43 2016
 *
 */

/* Include files */
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"
#include "heel_toe_planner/COM_Generation/COM_Boundary_Condition.h"
#include "heel_toe_planner/COM_Generation/COM_Generation.h"
#include "heel_toe_planner/COM_Generation/Cosine_Cosine_Interpolation.h"
#include "heel_toe_planner/COM_Generation/Forward_Kinematic.h"
#include "heel_toe_planner/COM_Generation/Gait_Generation.h"
#include "heel_toe_planner/COM_Generation/Quadratic_Cosine_Interpolation.h"
#include "heel_toe_planner/COM_Generation/X_DS_Differential_Equa_Solver.h"
#include "heel_toe_planner/COM_Generation/X_SS_Differential_Equa_Solver.h"
#include "heel_toe_planner/COM_Generation/Y_Correction.h"
#include "heel_toe_planner/COM_Generation/Y_DS_Differential_Equa_Solver.h"
#include "heel_toe_planner/COM_Generation/Y_End.h"
#include "heel_toe_planner/COM_Generation/Y_SS_Differential_Equa_Solver.h"
#include "heel_toe_planner/COM_Generation/ZMP_Boundary_Condition.h"
#include "heel_toe_planner/COM_Generation/ZMP_END_FITTING.h"
#include "heel_toe_planner/COM_Generation/ZMP_Generation.h"
#include <stdio.h>

/* Function Definitions */
void Forward_Kinematic(double theta_hip, double theta_knee, double theta_ankle,
  double lthigh, double lshank, double lfoot, double *x_cof, double *z_cof,
  double *x_heel, double *z_heel)
{
  double x_ankle;
  double z_ankle;

  /* Forward_Kinematic */
  /* Input: theta_hip, theta_knee, theta_ankle, lthigh, lshank, lfoot */
  /* Output(in HIP's frame): x_toe, z_toe, x_heel, z_heel, x_cof, z_cof */
  x_ankle = lthigh * sin(theta_hip) + lshank * sin(theta_hip + theta_knee);
  z_ankle = -lthigh * cos(theta_hip) - lshank * cos(theta_hip + theta_knee);
  *x_heel = x_ankle - 0.2 * lfoot * cos((theta_hip + theta_knee) - theta_ankle);
  *z_heel = z_ankle - 0.2 * lfoot * sin((theta_hip + theta_knee) - theta_ankle);
  *x_cof = x_ankle + 0.3 * lfoot * cos((theta_hip + theta_knee) - theta_ankle);
  *z_cof = z_ankle + 0.3 * lfoot * sin((theta_hip + theta_knee) - theta_ankle);
}

/* End of code generation (Forward_Kinematic.c) */
