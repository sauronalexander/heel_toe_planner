/*
 * COM_Boundary_Condition.c
 *
 * Code generation for function 'COM_Boundary_Condition'
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
void test_COM_Boundary_Condition()
{
    printf("Testing COM Boundary Condition\n");
}

void COM_Boundary_Condition(double lthigh, double lshank, double lfoot, double
  htar, double lstep, double z2, double x_init_pframe[2], double
  x_init_heel_pframe[2], double x_end_pframe[2], double x_end_heel_pframe[2],
  double theta_iss[3], double theta_ess[4])
{
  int i;
  static const double dv0[3] = { -1.5707963267948966, -1.5707963267948966, 0.0 };

  static const double dv1[4] = { 0.0, -1.5707963267948966, -1.5707963267948966,
    -1.5707963267948966 };

  double z_init_heel;
  double x_init_heel;
  double x_init_z;
  double x_init_x;
  for (i = 0; i < 3; i++) {
    theta_iss[i] = dv0[i];
  }

  for (i = 0; i < 4; i++) {
    theta_ess[i] = dv1[i];
  }

  /* This is to determine the Configurations during the transitions between double support and single support */
  /* Input:  */
  /* Output: x_init_pframe, x_end_pframe */
  /* { */
  NLP2_Solver(lthigh, lshank, lfoot, lstep, z2, htar, theta_iss);

  Forward_Kinematic(theta_iss[0], theta_iss[1], theta_iss[2], lthigh,
                    lshank, lfoot, &x_init_x, &x_init_z, &x_init_heel,
                    &z_init_heel, htar);
  x_init_pframe[0] = x_init_x;
  x_init_pframe[1] = x_init_z;
  x_init_heel_pframe[0] = x_init_heel;
  x_init_heel_pframe[1] = z_init_heel;
  double zi = -0.5*lfoot*sin(theta_iss[0]+theta_iss[1]);
  /* { */
  /*  theta_ess = fsolve(@(theta_ess) [lthigh*sin(theta_ess(1)) + lshank*sin(theta_ess(1)+theta_ess(2)) - htar*lfoot*cos(theta_ess(1)+theta_ess(2)-theta_ess(3)) + (lshank+lthigh)*sin(-theta_ess(4)) - (lstep - lfoot + lfoot*htar); */
  /*      lthigh*cos(theta_ess(1)) + lshank*cos(theta_ess(1)+theta_ess(2)) + htar*lfoot*sin(theta_ess(1)+theta_ess(2)-theta_ess(3)) - (lthigh + lshank) * cos(theta_ess(4)); */
  /*      (lthigh + lshank) * cos(theta_ess(4)) - z2; */
  /*      0.5*lfoot*sin(theta_ess(1)+theta_ess(2)-theta_ess(3)) - zi], theta0, optimset('Display','off', 'MaxFunEvals', 1000)); */
  /*  %} */
  NLP3_Solver(lthigh, lshank, lfoot, lstep, z2, htar, zi, theta_ess);

  Forward_Kinematic(theta_ess[0], theta_ess[1], theta_ess[2], lthigh,
                    lshank, lfoot, &x_init_x, &x_init_z, &x_init_heel,
                    &z_init_heel, htar);
  x_end_pframe[0] = x_init_x;
  x_end_pframe[1] = x_init_z;
  x_end_heel_pframe[0] = x_init_heel;
  x_end_heel_pframe[1] = z_init_heel;
}

/* End of code generation (COM_Boundary_Condition.c) */
