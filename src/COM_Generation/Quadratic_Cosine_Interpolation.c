/*
 * Quadratic_Cosine_Interpolation.c
 *
 * Code generation for function 'Quadratic_Cosine_Interpolation'
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

/* Function Declarations */
static double rt_atan2d_snf(double u0, double u1);

/* Function Definitions */
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2(b_u0, b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

void Quadratic_Cosine_Interpolation(double delta01, double delta12, double
  delta23, double y_zmp0, double y_zmp1, double y_zmp3, double y_zmp_prime0,
  double coef_y[6])
{
  int r1;
  static const signed char iv0[3] = { 1, 0, 0 };

  double A[9];
  static const signed char iv1[3] = { 0, 1, 0 };

  double c[3];
  int r2;
  int rtemp;
  int maxval;
  double b_idx_1;
  double b_idx_2;
  double y_zmp_prime1;
  double b[6];
  double delta13 = delta12 + delta23;
  (void)y_zmp3;

  /* Quadratic_Cosine_Interpolation */
  /* input: delta01, delta12, delta23, y_zmp0, y_zmp1, y_zmp3, y_zmp_prime0; */
  /* output: coef_y = [B1, w3, fai3, b0, b1, b2]'; */
  /*  t0<t<t1 */
  /* y_zmp1(t) = b0 + b1(t-t0) + b2(t-t0)^2; */
  /*  t1<t<t3 */
  /* y_zmp2(t) = B1cos(w3+fai3); */
  /* Initialization */
  /* check whether it is the end */
  /* first determine y_zmp1(t) */
  for (r1 = 0; r1 < 3; r1++) {
    A[3 * r1] = iv0[r1];
  }

  A[1] = 1.0;
  A[4] = delta01;
  A[7] = delta01 * delta01;
  for (r1 = 0; r1 < 3; r1++) {
    A[2 + 3 * r1] = iv1[r1];
  }

  c[0] = y_zmp0;
  c[1] = y_zmp1;
  c[2] = y_zmp_prime0;
  r1 = 0;
  r2 = 1;
  rtemp = 2;
  maxval = (int)A[0];
  if (1 > maxval) {
    r1 = 1;
    r2 = 0;
  }

  A[r2] /= A[r1];
  A[2] /= A[r1];
  A[3 + r2] -= A[r2] * A[3 + r1];
  A[5] -= A[2] * A[3 + r1];
  A[6 + r2] -= A[r2] * A[6 + r1];
  A[8] -= A[2] * A[6 + r1];
  if (fabs(A[5]) > fabs(A[3 + r2])) {
    rtemp = r2;
    r2 = 2;
  }

  A[3 + rtemp] /= A[3 + r2];
  A[6 + rtemp] -= A[3 + rtemp] * A[6 + r2];
  b_idx_1 = c[r2] - c[r1] * A[r2];
  b_idx_2 = ((c[rtemp] - c[r1] * A[rtemp]) - b_idx_1 * A[3 + rtemp]) / A[6 +
    rtemp];
  b_idx_1 -= b_idx_2 * A[6 + r2];
  b_idx_1 /= A[3 + r2];
  y_zmp_prime1 = b_idx_1 + 2.0 * b_idx_2 * delta01;

  /* determine y_zmp2(t) */
  /* w3 = fsolve(@(w3) (y_zmp1 * cos(delta13*w3) + (y_zmp_prime1/w3) * sin(w3*delta13) - y_zmp3), 0.1, optimset('Display','off')); */
  /* [w3] = Nonlinear_Eqn_Solver(y_zmp1, delta13, y_zmp_prime1, y_zmp3); */
  double w3 = NLP1_Solver(y_zmp1, delta13, y_zmp_prime1, y_zmp3);

  if (y_zmp1 > 0.0) {
    y_zmp_prime1 = rt_atan2d_snf(-y_zmp_prime1, w3 * y_zmp1);
  } else {
    y_zmp_prime1 = rt_atan2d_snf(y_zmp_prime1, -1.0 * w3 * y_zmp1);
  }

  b[0] = ((c[r1] - b_idx_2 * A[6 + r1]) - b_idx_1 * A[3 + r1]) / A[r1];
  b[1] = b_idx_1;
  b[2] = b_idx_2;
  b[3] = y_zmp1 / cos(y_zmp_prime1);
  b[4] = w3;
  b[5] = y_zmp_prime1 - w3 * delta01;
  for (r1 = 0; r1 < 6; r1++) {
    coef_y[r1] = b[r1];
  }
}

/* End of code generation (Quadratic_Cosine_Interpolation.c) */
