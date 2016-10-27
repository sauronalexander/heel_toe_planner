/*
 * Y_Correction.c
 *
 * Code generation for function 'Y_Correction'
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
void Y_Correction(double delta01, double y_star, double zmp_y0, double
                  zmp_prime_y0, double delta_y[5])
{
  double y;
  double dv8[25];
  int i8;
  static const signed char iv2[5] = { 0, 0, 0, 0, 1 };

  static const signed char iv3[5] = { 0, 0, 0, 1, 0 };

  /* Y_Correction */
  y = delta01 / 2.0;
  delta_y[0] = 0.0;
  delta_y[1] = 0.0;
  delta_y[2] = 0.0;
  delta_y[3] = 0.0;
  delta_y[4] = (y_star - zmp_y0) - delta01 * zmp_prime_y0 / 4.0;
  for (i8 = 0; i8 < 5; i8++) {
    dv8[5 * i8] = iv2[i8];
    dv8[1 + 5 * i8] = iv3[i8];
  }

  dv8[2] = rt_powd_snf(delta01, 4.0);
  dv8[7] = rt_powd_snf(delta01, 3.0);
  dv8[12] = delta01 * delta01;
  dv8[17] = delta01;
  dv8[22] = 1.0;
  dv8[3] = 4.0 * rt_powd_snf(delta01, 3.0);
  dv8[8] = 3.0 * (delta01 * delta01);
  dv8[13] = 2.0 * delta01;
  dv8[18] = 1.0;
  dv8[23] = 0.0;
  dv8[4] = rt_powd_snf(delta01 / 2.0, 4.0);
  dv8[9] = rt_powd_snf(delta01 / 2.0, 3.0);
  dv8[14] = y * y;
  dv8[19] = delta01 / 2.0;
  dv8[24] = 1.0;
  b_mldivide(dv8, delta_y);
}

/* End of code generation (Y_Correction.c) */
