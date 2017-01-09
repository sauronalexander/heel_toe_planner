/*
 * Y_End.c
 *
 * Code generation for function 'Y_End'
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
#include "heel_toe_planner/COM_Generation/mldivide.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_rtwutil.h"
#include <stdio.h>

/* Function Definitions */
void Y_End(double g, double h, double A4, double A3, double A2, double A1,
           double A0, double Constant, double y_init, double y_end, double tend,
           double *alpha, double *beta, double *b_gamma, double *fai, double
           *ita, double *Constant_prime, double *C1, double *C2)
{
  double x[5];
  double b_g[25];
  double dv6[4];
  int i5;
  double b_y_init[2];
  double y[2];

  /* Solve Linear ODE for Y component during end phase */
  x[0] = A4;
  x[1] = A3;
  x[2] = A2;
  x[3] = A1;
  x[4] = A0;
  b_g[0] = -g / h;
  b_g[5] = 0.0;
  b_g[10] = 0.0;
  b_g[15] = 0.0;
  b_g[20] = 0.0;
  b_g[1] = 0.0;
  b_g[6] = -g / h;
  b_g[11] = 0.0;
  b_g[16] = 0.0;
  b_g[21] = 0.0;
  b_g[2] = 12.0;
  b_g[7] = 0.0;
  b_g[12] = -g / h;
  b_g[17] = 0.0;
  b_g[22] = 0.0;
  b_g[3] = 0.0;
  b_g[8] = 6.0;
  b_g[13] = 0.0;
  b_g[18] = -g / h;
  b_g[23] = 0.0;
  b_g[4] = 0.0;
  b_g[9] = 0.0;
  b_g[14] = 2.0;
  b_g[19] = 0.0;
  b_g[24] = -g / h;
  b_mldivide(b_g, x);
  *alpha = x[0];
  *beta = x[1];
  *b_gamma = x[2];
  *fai = x[3];
  *ita = x[4];
  *Constant_prime = -Constant;
  for (i5 = 0; i5 < 2; i5++) {
    dv6[i5 << 1] = 1.0;
  }

  dv6[1] = exp(-sqrt(g / h) * tend);
  dv6[3] = exp(sqrt(g / h) * tend);
  b_y_init[0] = (y_init - (-Constant)) - x[4];
  b_y_init[1] = (((((y_end - (-Constant)) - x[0] * rt_powd_snf(tend, 4.0)) - x[1]
                   * rt_powd_snf(tend, 3.0)) - x[2] * (tend * tend)) - x[3] *
                 tend) - x[4];
  mldivide(dv6, b_y_init, y);
  *C1 = y[0];
  *C2 = y[1];
}

/* End of code generation (Y_End.c) */
