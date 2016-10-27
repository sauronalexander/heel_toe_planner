/*
 * Cosine_Cosine_Interpolation.c
 *
 * Code generation for function 'Cosine_Cosine_Interpolation'
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
#include "heel_toe_planner/COM_Generation/fprintf.h"
#include <stdio.h>

/* Function Definitions */
void Cosine_Cosine_Interpolation(double delta01, double delta12, double delta23,
  double x_zmp0, double x_zmp1, double x_zmp3, double coef_x[8])
{
  double O1;
  double O2;
  double temp1;
  double temp2;

  /* Cosine_Cosine_Interpolation */
  /* input: (delta01, delta12, delta03, zmp_x_0, zmp_x_1, zmp_x_3) */
  /* output: coef_x = [A1, w1, fai1, O1, A2, w2, fai2, O2]'; */
  /*  t0<t<t1 */
  /* x_zmp1(t) = A1*cos(w1*t + fai1) + O1; */
  /*  t1<t<t3 */
  /* x_zmp2(t) = A2*cos(w2*t + fai2) + O2; */
  /* Initialization */
  O1 = 0.5 * (x_zmp0 + x_zmp1);
  O2 = 0.5 * (x_zmp1 + x_zmp3);

  /* solve w1, w2 */
  temp1 = (x_zmp1 - O1) / (x_zmp0 - O1);
  temp2 = (x_zmp3 - O2) / (x_zmp1 - O2);
  if (temp1 - 1.0 > 0.05) {
    b_fprintf();
  } else {
    if (-1.0 - temp2 > 0.05) {
      b_fprintf();
    }
  }

  if (temp1 > 1.0) {
    temp1 = 1.0;
  } else {
    if (temp1 < -1.0) {
      temp1 = -1.0;
    }
  }

  if (temp2 > 1.0) {
    temp2 = 1.0;
  } else {
    if (temp2 < -1.0) {
      temp2 = -1.0;
    }
  }

  temp2 = acos(temp2) / (delta12 + delta23);
  coef_x[0] = -(x_zmp0 - O1);
  coef_x[1] = acos(temp1) / delta01;
  coef_x[2] = 3.1415926535897931;
  coef_x[3] = O1;
  coef_x[4] = -(x_zmp1 - O2);
  coef_x[5] = temp2;
  coef_x[6] = 3.1415926535897931 - temp2 * delta01;
  coef_x[7] = O2;
}

/* End of code generation (Cosine_Cosine_Interpolation.c) */
