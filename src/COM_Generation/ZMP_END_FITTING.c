/*
 * ZMP_END_FITTING.c
 *
 * Code generation for function 'ZMP_END_FITTING'
 *
 * C source code generated on: Wed Oct 12 13:19:44 2016
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
#include "heel_toe_planner/COM_Generation/mldivide.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_rtwutil.h"
#include <stdio.h>

/* Function Definitions */
void ZMP_END_FITTING(double delta01, double y_zmp0, double y_zmp1, double
                     y_zmp_prime0, double coef_y[5])
{
  double dv10[25];
  int i11;
  static const signed char iv4[5] = { 0, 0, 0, 0, 1 };

  static const signed char iv5[5] = { 0, 0, 0, 1, 0 };

  /* ZMP_END_FITTING */
  /* This is used to stablize the robot from a moving state to a standing state */
  /* At t=0, some force or torque is applied and Y_zmp behaves like an impulse */
  /* respose to the input */
  /* y(t) = A + B*exp(-alpha*t); */
  /* Input: tend, y_zmp0, y_zmp1, y_zmp_prime0; */
  coef_y[0] = y_zmp0;
  coef_y[1] = y_zmp_prime0;
  coef_y[2] = y_zmp1;
  coef_y[3] = 0.0;
  coef_y[4] = 0.0;
  for (i11 = 0; i11 < 5; i11++) {
    dv10[5 * i11] = iv4[i11];
    dv10[1 + 5 * i11] = iv5[i11];
  }

  dv10[2] = rt_powd_snf(delta01, 4.0);
  dv10[7] = rt_powd_snf(delta01, 3.0);
  dv10[12] = delta01 * delta01;
  dv10[17] = delta01;
  dv10[22] = 1.0;
  dv10[3] = 4.0 * rt_powd_snf(delta01, 3.0);
  dv10[8] = 3.0 * (delta01 * delta01);
  dv10[13] = 2.0 * delta01;
  dv10[18] = 1.0;
  dv10[23] = 0.0;
  dv10[4] = 12.0 * (delta01 * delta01);
  dv10[9] = 6.0 * delta01;
  dv10[14] = 2.0;
  dv10[19] = 0.0;
  dv10[24] = 0.0;
  b_mldivide(dv10, coef_y);
}

/* End of code generation (ZMP_END_FITTING.c) */
