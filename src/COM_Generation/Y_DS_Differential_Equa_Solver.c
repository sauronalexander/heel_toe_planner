/*
 * Y_DS_Differential_Equa_Solver.c
 *
 * Code generation for function 'Y_DS_Differential_Equa_Solver'
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
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_emxutil.h"
#include "heel_toe_planner/COM_Generation/mldivide.h"

/* Function Definitions */
void Y_DS_Differential_Equa_Solver(double Start, double End, const double
  t_data[1151], const int t_size[2], double g, double h, double lfoot, double
  tds, const double theta_ess[4], const double theta_iss[3], double m1, double
  m2, double m3, double b_y1, double y3, double B, double wb, double faib,
  double t0, double y2_init, double y2_end, emxArray_real_T *y2)
{
  double x[4];
  double b_t0[16];
  double Constant_prime;
  double Af1s_prime;
  double Ag1s_prime;
  double Awbc_prime;
  double Ag1_minus_prime;
  double Af1_plus_prime;
  double Af1_minus_prime;
  double Ag1_plus_prime;
  double dv6[4];
  double b_y2_init[2];
  double C[2];
  int i7;
  int loop_ub;
  double i;
  (void)t_size;

  /* Solving Linear ODE for Y component during Double Support Phase */
  x[0] = (theta_ess[0] + theta_ess[1]) - theta_ess[2];
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = theta_iss[0] + theta_iss[1];
  b_t0[0] = t0;
  b_t0[4] = 1.0;
  b_t0[8] = 0.0;
  b_t0[12] = 0.0;
  b_t0[1] = t0 + tds;
  b_t0[5] = 1.0;
  b_t0[9] = 0.0;
  b_t0[13] = 0.0;
  b_t0[2] = 0.0;
  b_t0[6] = 0.0;
  b_t0[10] = t0;
  b_t0[14] = 1.0;
  b_t0[3] = 0.0;
  b_t0[7] = 0.0;
  b_t0[11] = t0 + tds;
  b_t0[15] = 1.0;
  c_mldivide(b_t0, x);
  Constant_prime = -(m1 * b_y1 + m3 * y3) / m2;

  /* Sine Function with w = f1 with angle f0 */
  Af1s_prime = -0.5 * lfoot * (x[0] * x[0]) * m1 * b_y1 / (m2 * h) / (-(x[0] *
    x[0]) - g / h);

  /* Sine Function with w = g1 with angle g0 */
  Ag1s_prime = -0.5 * lfoot * (x[2] * x[2]) * m3 * y3 / (m2 * h) / (-(x[2] * x[2])
    - g / h);

  /* Consine Function with w = wb with angle faib */
  Awbc_prime = -((m1 + m2) + m3) * g * B / (m2 * h) / (-(wb * wb) - g / h);

  /* Sine theta * Cosine fai for m1: ~sine */
  Ag1_minus_prime = x[0] + wb;
  Af1_plus_prime = 0.25 * m1 * B * (x[0] * x[0]) * lfoot / (m2 * h) /
    (-(Ag1_minus_prime * Ag1_minus_prime) - g / h);
  Ag1_minus_prime = x[0] - wb;
  Af1_minus_prime = 0.25 * m1 * B * (x[0] * x[0]) * lfoot / (m2 * h) /
    (-(Ag1_minus_prime * Ag1_minus_prime) - g / h);

  /* Sine theta * Cosine fai for m3: ~sine */
  Ag1_minus_prime = x[2] + wb;
  Ag1_plus_prime = 0.25 * (x[2] * x[2]) * lfoot * m3 * B / (m2 * h) /
    (-(Ag1_minus_prime * Ag1_minus_prime) - g / h);
  Ag1_minus_prime = x[2] - wb;
  Ag1_minus_prime = -0.25 * (x[2] * x[2]) * lfoot * m3 * B / (m2 * h) /
    (-(Ag1_minus_prime * Ag1_minus_prime) - g / h);

  /* y2(t) = Af1s_prime*sin(f1*t+f0) + Ag1s_prime*sin(g1*t+g0) ... */
  /*         + Awbc_prime*cos(wb*t+faib)+ ... */
  /*         + Af1_plus_prime*sin((f1+wb)*t+(f0+faib)) ... */
  /*         + Af1_minus_prime*sin((f1-wb)*t+(f0-faib)) ... */
  /*         + Ag1_plus_prime*sin((g1+wb)*t+(g0+faib)) ... */
  /*         + Af1_minus_prime*sin((g1-wb)*t+(g0-faib)); */
  dv6[0] = exp(-sqrt(g / h) * t0);
  dv6[2] = exp(sqrt(g / h) * t0);
  dv6[1] = exp(-sqrt(g / h) * (t0 + tds));
  dv6[3] = exp(sqrt(g / h) * (t0 + tds));
  b_y2_init[0] = y2_init - (((((((Af1s_prime * sin(x[0] * t0 + x[1]) +
    Ag1s_prime * sin(x[2] * t0 + x[3])) + Awbc_prime * cos(wb * t0 + faib)) +
    Af1_plus_prime * sin((x[0] + wb) * t0 + (x[1] + faib))) + Af1_minus_prime *
    sin((x[0] - wb) * t0 + (x[1] - faib))) + Ag1_plus_prime * sin((x[2] + wb) *
    t0 + (x[3] + faib))) + Af1_minus_prime * sin((x[2] - wb) * t0 + (x[3] - faib)))
    + Constant_prime);
  b_y2_init[1] = y2_end - (((((((Af1s_prime * sin(x[0] * (t0 + tds) + x[1]) +
    Ag1s_prime * sin(x[2] * (t0 + tds) + x[3])) + Awbc_prime * cos(wb * (t0 +
    tds) + faib)) + Af1_plus_prime * sin((x[0] + wb) * (t0 + tds) + (x[1] + faib)))
    + Af1_minus_prime * sin((x[0] - wb) * (t0 + tds) + (x[1] - faib))) +
    Ag1_plus_prime * sin((x[2] + wb) * (t0 + tds) + (x[3] + faib))) +
    Af1_minus_prime * sin((x[2] - wb) * (t0 + tds) + (x[3] - faib))) +
    Constant_prime);
  mldivide(dv6, b_y2_init, C);
  i7 = y2->size[0] * y2->size[1];
  y2->size[0] = 1;
  y2->size[1] = (int)((Start - End) + 1.0);
  emxEnsureCapacity((emxArray__common *)y2, i7, (int)sizeof(double));
  loop_ub = (int)((Start - End) + 1.0);
  for (i7 = 0; i7 < loop_ub; i7++) {
    y2->data[i7] = 0.0;
  }

  i7 = (int)((End + 1.0) + (1.0 - (Start + 1.0)));
  for (loop_ub = 0; loop_ub < i7; loop_ub++) {
    i = (Start + 1.0) + (double)loop_ub;
    y2->data[(int)(i - Start) - 1] = ((((((((C[0] * exp(-sqrt(g / h) * t_data
      [(int)i - 1]) + C[1] * exp(sqrt(g / h) * t_data[(int)i - 1])) + Af1s_prime
      * sin(x[0] * t_data[(int)i - 1] + x[1])) + Ag1s_prime * sin(x[2] * t_data
      [(int)i - 1] + x[3])) + Awbc_prime * cos(wb * t_data[(int)i - 1] + faib))
      + Af1_plus_prime * sin((x[0] + wb) * t_data[(int)i - 1] + (x[1] + faib)))
      + Af1_minus_prime * sin((x[0] - wb) * t_data[(int)i - 1] + (x[1] - faib)))
      + Ag1_plus_prime * sin((x[2] + wb) * t_data[(int)i - 1] + (x[3] + faib)))
      + Ag1_minus_prime * sin((x[2] - wb) * t_data[(int)i - 1] + (x[3] - faib)))
      + Constant_prime;
  }
}

void b_Y_DS_Differential_Equa_Solver(double Start, double End, const
  emxArray_real_T *t, double g, double h, double lfoot, double tds, const double
  theta_ess[4], const double theta_iss[3], double m1, double m2, double m3,
  double b_y1, double y3, double B, double wb, double faib, double t0, double
  y2_init, double y2_end, emxArray_real_T *y2)
{
  double x[4];
  double b_t0[16];
  double Constant_prime;
  double Af1s_prime;
  double Ag1s_prime;
  double Awbc_prime;
  double Ag1_minus_prime;
  double Af1_plus_prime;
  double Af1_minus_prime;
  double Ag1_plus_prime;
  double dv3[4];
  double b_y2_init[2];
  double C[2];
  int i4;
  int loop_ub;
  double i;
  /* Solving Linear ODE for Y component during Double Support Phase */
  x[0] = (theta_ess[0] + theta_ess[1]) - theta_ess[2];
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = theta_iss[0] + theta_iss[1];
  b_t0[0] = t0;
  b_t0[4] = 1.0;
  b_t0[8] = 0.0;
  b_t0[12] = 0.0;
  b_t0[1] = t0 + tds;
  b_t0[5] = 1.0;
  b_t0[9] = 0.0;
  b_t0[13] = 0.0;
  b_t0[2] = 0.0;
  b_t0[6] = 0.0;
  b_t0[10] = t0;
  b_t0[14] = 1.0;
  b_t0[3] = 0.0;
  b_t0[7] = 0.0;
  b_t0[11] = t0 + tds;
  b_t0[15] = 1.0;
  c_mldivide(b_t0, x);
  Constant_prime = -(m1 * b_y1 + m3 * y3) / m2;
  /* Sine Function with w = f1 with angle f0 */
  Af1s_prime = -0.5 * lfoot * (x[0] * x[0]) * m1 * b_y1 / (m2 * h) / (-(x[0] *
    x[0]) - g / h);

  /* Sine Function with w = g1 with angle g0 */
  Ag1s_prime = -0.5 * lfoot * (x[2] * x[2]) * m3 * y3 / (m2 * h) / (-(x[2] * x[2])
    - g / h);

  /* Consine Function with w = wb with angle faib */
  Awbc_prime = -((m1 + m2) + m3) * g * B / (m2 * h) / (-(wb * wb) - g / h);

  /* Sine theta * Cosine fai for m1: ~sine */
  Ag1_minus_prime = x[0] + wb;
  Af1_plus_prime = 0.25 * m1 * B * (x[0] * x[0]) * lfoot / (m2 * h) /
    (-(Ag1_minus_prime * Ag1_minus_prime) - g / h);
  Ag1_minus_prime = x[0] - wb;
  Af1_minus_prime = 0.25 * m1 * B * (x[0] * x[0]) * lfoot / (m2 * h) /
    (-(Ag1_minus_prime * Ag1_minus_prime) - g / h);

  /* Sine theta * Cosine fai for m3: ~sine */
  Ag1_minus_prime = x[2] + wb;
  Ag1_plus_prime = 0.25 * (x[2] * x[2]) * lfoot * m3 * B / (m2 * h) /
    (-(Ag1_minus_prime * Ag1_minus_prime) - g / h);
  Ag1_minus_prime = x[2] - wb;
  Ag1_minus_prime = -0.25 * (x[2] * x[2]) * lfoot * m3 * B / (m2 * h) /
    (-(Ag1_minus_prime * Ag1_minus_prime) - g / h);
  /* y2(t) = Af1s_prime*sin(f1*t+f0) + Ag1s_prime*sin(g1*t+g0) ... */
  /*         + Awbc_prime*cos(wb*t+faib)+ ... */
  /*         + Af1_plus_prime*sin((f1+wb)*t+(f0+faib)) ... */
  /*         + Af1_minus_prime*sin((f1-wb)*t+(f0-faib)) ... */
  /*         + Ag1_plus_prime*sin((g1+wb)*t+(g0+faib)) ... */
  /*         + Af1_minus_prime*sin((g1-wb)*t+(g0-faib)); */
  dv3[0] = exp(-sqrt(g / h) * t0);
  dv3[2] = exp(sqrt(g / h) * t0);
  dv3[1] = exp(-sqrt(g / h) * (t0 + tds));
  dv3[3] = exp(sqrt(g / h) * (t0 + tds));
  b_y2_init[0] = y2_init - (((((((Af1s_prime * sin(x[0] * t0 + x[1]) +
    Ag1s_prime * sin(x[2] * t0 + x[3])) + Awbc_prime * cos(wb * t0 + faib)) +
    Af1_plus_prime * sin((x[0] + wb) * t0 + (x[1] + faib))) + Af1_minus_prime *
    sin((x[0] - wb) * t0 + (x[1] - faib))) + Ag1_plus_prime * sin((x[2] + wb) *
    t0 + (x[3] + faib))) + Af1_minus_prime * sin((x[2] - wb) * t0 + (x[3] - faib)))
    + Constant_prime);
  b_y2_init[1] = y2_end - (((((((Af1s_prime * sin(x[0] * (t0 + tds) + x[1]) +
    Ag1s_prime * sin(x[2] * (t0 + tds) + x[3])) + Awbc_prime * cos(wb * (t0 +
    tds) + faib)) + Af1_plus_prime * sin((x[0] + wb) * (t0 + tds) + (x[1] + faib)))
    + Af1_minus_prime * sin((x[0] - wb) * (t0 + tds) + (x[1] - faib))) +
    Ag1_plus_prime * sin((x[2] + wb) * (t0 + tds) + (x[3] + faib))) +
    Af1_minus_prime * sin((x[2] - wb) * (t0 + tds) + (x[3] - faib))) +
    Constant_prime);
  mldivide(dv3, b_y2_init, C);
  i4 = y2->size[0] * y2->size[1];
  y2->size[0] = 1;
  y2->size[1] = (int)((End - Start) + 1.0);
  emxEnsureCapacity((emxArray__common *)y2, i4, (int)sizeof(double));
  loop_ub = (int)((Start - End) + 1.0);
  for (i4 = 0; i4 < loop_ub; i4++) {
    y2->data[i4] = 0.0;
  }
  i4 = (int)((End + 1.0) + (1.0 - (Start + 1.0)));

  for (loop_ub = 0; loop_ub < i4; loop_ub++) {
    i = (Start + 1.0) + (double)loop_ub;
    y2->data[(int)(i - Start) - 1] = ((((((((C[0] * exp(-sqrt(g / h) * t->data
      [(int)i - 1]) + C[1] * exp(sqrt(g / h) * t->data[(int)i - 1])) +
      Af1s_prime * sin(x[0] * t->data[(int)i - 1] + x[1])) + Ag1s_prime * sin(x
      [2] * t->data[(int)i - 1] + x[3])) + Awbc_prime * cos(wb * t->data[(int)i
      - 1] + faib)) + Af1_plus_prime * sin((x[0] + wb) * t->data[(int)i - 1] +
      (x[1] + faib))) + Af1_minus_prime * sin((x[0] - wb) * t->data[(int)i - 1]
      + (x[1] - faib))) + Ag1_plus_prime * sin((x[2] + wb) * t->data[(int)i - 1]
      + (x[3] + faib))) + Ag1_minus_prime * sin((x[2] - wb) * t->data[(int)i - 1]
      + (x[3] - faib))) + Constant_prime;
  }
}

/* End of code generation (Y_DS_Differential_Equa_Solver.c) */
