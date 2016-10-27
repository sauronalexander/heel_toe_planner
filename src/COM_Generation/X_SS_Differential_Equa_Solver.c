/*
 * X_SS_Differential_Equa_Solver.c
 *
 * Code generation for function 'X_SS_Differential_Equa_Solver'
 *
 * C source code generated on: Wed Oct 19 16:30:45 2016
 *
 */

/* Include files */
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"
#include "heel_toe_planner/COM_Generation/COM_Generation.h"
#include "heel_toe_planner/COM_Generation/X_DS_Differential_Equa_Solver.h"
#include "heel_toe_planner/COM_Generation/X_SS_Differential_Equa_Solver.h"
#include "heel_toe_planner/COM_Generation/Y_DS_Differential_Equa_Solver.h"
#include "heel_toe_planner/COM_Generation/Y_SS_Differential_Equa_Solver.h"
#include "heel_toe_planner/COM_Generation/mldivide.h"

/* Function Definitions */
void X_SS_Differential_Equa_Solver(double g, double h, double A1, double w1,
  double fai1, double A3, double w3, double fai3, double Constant, double x_init,
  double x_end, double tend, double *A1_prime, double *A3_prime, double
  *Constant_prime, double *C1, double *C2)
{
  double dv0[4];
  int i1;
  double b_x_init[2];
  double x[2];

  /* Solving Linear ODE for X component during Single Support Phase */
  *A1_prime = -A1 / (w1 * w1 + g / h);
  *A3_prime = -A3 / (w3 * w3 + g / h);
  *Constant_prime = -Constant;
  for (i1 = 0; i1 < 2; i1++) {
    dv0[i1 << 1] = 1.0;
  }

  dv0[1] = exp(-sqrt(g / h) * tend);
  dv0[3] = exp(sqrt(g / h) * tend);
  b_x_init[0] = ((x_init - (-Constant)) - *A1_prime * cos(fai1)) - *A3_prime *
    cos(fai3);
  b_x_init[1] = ((x_end - (-Constant)) - *A1_prime * cos(w1 * tend + fai1)) -
    *A3_prime * cos(w3 * tend + fai3);
  mldivide(dv0, b_x_init, x);
  *C1 = x[0];
  *C2 = x[1];
}

/* End of code generation (X_SS_Differential_Equa_Solver.c) */
