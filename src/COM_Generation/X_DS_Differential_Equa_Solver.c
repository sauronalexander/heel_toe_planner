/*
 * X_DS_Differential_Equa_Solver.c
 *
 * Code generation for function 'X_DS_Differential_Equa_Solver'
 *
 * C source code generated on: Sat Oct 29 20:17:16 2016
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
#include "assert.h"

/* Function Definitions */
void X_DS_Differential_Equa_Solver(double Start, double End, emxArray_real_T * t, 
	double g, double h, double lfoot, double
  tds, const double theta_ess[4], const double theta_iss[3], double m1, double
  m2, double m3, double ze, double A2, double w2, double fai2, double O2, double
  t0, double x1_0, double x3_0, double x2_init, double x2_end,
  emxArray_real_T *x1, emxArray_real_T *x2, emxArray_real_T *x3, emxArray_real_T
  *z1, emxArray_real_T *z3, emxArray_real_T *theta_1, emxArray_real_T *theta_3)
{
  double x[4];
  double b_t0[16];
  double Af1c_prime;
  double Af1s_prime;
  double a21;
  double Af2s_prime;
  double Cf1_prime;
  double Ag1c_prime;
  double Ag1s_prime;
  double Ag2s_prime;
  double Cg1_prime;
  double Aw2_prime;
  double Af1_plus_prime;
  double Af1_minus_prime;
  double Cw2_prime;
  double Ag1_plus_prime;
  double Ag1_minus_prime;
  double E[4];
  double f[2];
  int r1;
  int r2;
  double a22;
  double z;


  /* Solving Linear ODE for X component during Double Support Phase */
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

  /* Cosine Function with w = f1 with angle f0 */
  /* m1*ze*(0.5*f1^2)*lfoot/(m2*h) - 0.5*m1*lfoot*g/(m2*h); */
  Af1c_prime = (m1 * lfoot * (g + x[0] * x[0] * ze) / (2.0 * m2 * h) - m1 *
                lfoot * (x[0] * x[0]) * x1_0 / (2.0 * m2 * h)) / (-(x[0] * x[0])
    - g / h);

  /* Sine Function with w = f1 with angle f0 */
  /* 0.5*m1*f1^2*lfoot*x1_0/(m2*h); */
  Af1s_prime = O2 * m1 * lfoot * (x[0] * x[0]) / (2.0 * m2 * h) / (-(x[0] * x[0])
    - g / h);

  /* Sine Function with w = 2f1 with angle 2f0 */
  /* 0.5^3*m1*lfoot^2*f1^2/(m2*h) - 0.5*m1*O2*f1^2*lfoot/(m2*h); */
  a21 = 2.0 * x[0];
  Af2s_prime = -0.125 * m1 * (lfoot * lfoot) * (x[0] * x[0]) / (m2 * h) / (-(a21
    * a21) - g / h);

  /* Constant involved in m1x1(z1''+g) */
  /* -m1*x1_0*g/(m2*h); */
  Cf1_prime = -(h / g) * (m1 * x1_0 * g / (m2 * h));

  /* Cosine Function with w = g1 with angle g0 */
  /* m3*ze*(0.5*g1^2)*lfoot/(m2*h) + 0.5*m3*lfoot*g/(m2*h); */
  Ag1c_prime = -m3 * lfoot * (g + x[2] * x[2] * ze) / (2.0 * m2 * h) / (-(x[2] *
    x[2]) - g / h);

  /* Sine Function with w = g1 with angle g0 */
  /* 0.5*m3*g1^2*lfoot*x3_0/(m2*h) - 0.5*m3*O2*g1^2*lfoot/(m2*h); */
  Ag1s_prime = (-m3 * lfoot * (x[2] * x[2]) * O2 / (2.0 * m2 * h) + m3 * x3_0 *
                lfoot * (x[2] * x[2]) / (2.0 * m2 * h)) / (-(x[2] * x[2]) - g /
    h);

  /* Sine Function with w = 2g1 with angle 2g0 */
  /* -0.5^3*m3*lfoot^2*g1^2/(m2*h); */
  a21 = 2.0 * x[2];
  Ag2s_prime = -0.125 * m3 * (lfoot * lfoot) * (x[2] * x[2]) / (m2 * h) / (-(a21
    * a21) - g / h);

  /* Constant involved in m1x1(z1''+g) */
  /* -m1*x3_0*g/(m2*h); */
  Cg1_prime = -(h / g) * (m1 * x3_0 * g / (m2 * h));

  /* Cosine Function with w = w2 with angle fai2 */
  Aw2_prime = -((A2 * m1 * g / (m2 * h) + A2 * m2 * g / (m2 * h)) + A2 * m3 * g /
                (m2 * h)) / (-(w2 * w2) - g / h);

  /* Sine theta * Cosine fai for m1: ~sine */
  a21 = x[0] + w2;
  Af1_plus_prime = 0.5 * m1 * A2 * (x[0] * x[0]) * lfoot / (2.0 * m2 * h) /
    (-(a21 * a21) - g / h);
  a21 = x[0] - w2;
  Af1_minus_prime = 0.5 * m1 * A2 * (x[0] * x[0]) * lfoot / (2.0 * m2 * h) /
    (-(a21 * a21) - g / h);

  /* Constant involved in xzmpm1(z1''+g) */
  Cw2_prime = -(h / g) * (-((m1 + m2) + m3) * O2 * g / (m2 * h));

  /* Sine theta * Cosine fai for m3: ~sine */
  a21 = x[2] + w2;
  Ag1_plus_prime = -0.5 * (x[2] * x[2]) * lfoot * m3 * A2 / (2.0 * m2 * h) /
    (-(a21 * a21) - g / h);
  a21 = x[2] - w2;
  Ag1_minus_prime = -0.5 * (x[2] * x[2]) * lfoot * m3 * A2 / (2.0 * m2 * h) /
    (-(a21 * a21) - g / h);

  /*  Adding Everything Up */
  /* Particular Solution */
  /* x2(t) = Af1c_prime*cos(f1*t+f0) + Af1s_prime*sin(f1*t+f0) ... */
  /*         + Af2s_prime*sin(2*f1*t+2*f0) + Cf1_prime ... */
  /*         + Ag1c_prime*cos(g1*t+g0) + Ag1s_prime*sin(g1*t+g0) ... */
  /*         + Ag2s_prime*sin(2*g1*t+2*g0) + Cg1_prime ... */
  /*         + Aw2*cos(w2*t+fai2) + Cw2_prime ... */
  /*         + Af1_plus_prime*sin((f1+w2)*t+(f0+fai2)) ... */
  /*         + Af1_minus_prime*sin((f1-w2)*t+(f0-fai2)) ... */
  /*         + Ag1_plus_prime*sin((g1+w2)*t+(g0+fai2)) ... */
  /*         + Af1_minus_prime*sin((g1-w2)*t+(g0-fai2)); */
  E[0] = exp(-sqrt(g / h) * t0);
  E[2] = exp(sqrt(g / h) * t0);
  E[1] = exp(-sqrt(g / h) * (t0 + tds));
  E[3] = exp(sqrt(g / h) * (t0 + tds));
  f[0] = x2_init - (((((((((((((Af1c_prime * cos(x[0] * t0 + x[1]) + Af1s_prime *
    sin(x[0] * t0 + x[1])) + Af2s_prime * sin(2.0 * x[0] * t0 + 2.0 * x[1])) +
    Cf1_prime) + Ag1c_prime * cos(x[2] * t0 + x[3])) + Ag1s_prime * sin(x[2] *
    t0 + x[3])) + Ag2s_prime * sin(2.0 * x[2] * t0 + 2.0 * x[3])) + Cg1_prime) +
    Aw2_prime * cos(w2 * t0 + fai2)) + Cw2_prime) + Af1_plus_prime * sin((x[0] +
    w2) * t0 + (x[1] + fai2))) + Af1_minus_prime * sin((x[0] - w2) * t0 + (x[1]
    - fai2))) + Ag1_plus_prime * sin((x[2] + w2) * t0 + (x[3] + fai2))) +
                    Ag1_minus_prime * sin((x[2] - w2) * t0 + (x[3] - fai2)));
  f[1] = x2_end - (((((((((((((Af1c_prime * cos(x[0] * (t0 + tds) + x[1]) +
    Af1s_prime * sin(x[0] * (t0 + tds) + x[1])) + Af2s_prime * sin(2.0 * x[0] *
                               (t0 + tds) + 2.0 * x[1])) + Cf1_prime) +
    Ag1c_prime * cos(x[2] * (t0 + tds) + x[3])) + Ag1s_prime * sin(x[2] * (t0 +
    tds) + x[3])) + Ag2s_prime * sin(2.0 * x[2] * (t0 + tds) + 2.0 * x[3])) +
    Cg1_prime) + Aw2_prime * cos(w2 * (t0 + tds) + fai2)) + Cw2_prime) +
                      Af1_plus_prime * sin((x[0] + w2) * (t0 + tds) + (x[1] +
    fai2))) + Af1_minus_prime * sin((x[0] - w2) * (t0 + tds) + (x[1] - fai2))) +
                    Ag1_plus_prime * sin((x[2] + w2) * (t0 + tds) + (x[3] + fai2)))
                   + Ag1_minus_prime * sin((x[2] - w2) * (t0 + tds) + (x[3] -
    fai2)));
  if (E[1] > E[0]) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = E[r2] / E[r1];
  a22 = E[2 + r2] - a21 * E[2 + r1];
  z = (f[r2] - f[r1] * a21) / a22;
  a21 = (f[r1] - (f[r2] - f[r1] * a21) / a22 * E[2 + r1]) / E[r1];
  r2 = x1->size[0] * x1->size[1];
  x1->size[0] = 1;
  x1->size[1] = (int)((End - Start) + 1.0);
  emxEnsureCapacity((emxArray__common *)x1, r2, (int)sizeof(double));
  r1 = (int)((End - Start) + 1.0);
  for (r2 = 0; r2 < r1; r2++) {
    x1->data[r2] = 0.0;
  }

  r2 = x2->size[0] * x2->size[1];
  x2->size[0] = 1;
  x2->size[1] = (int)((End - Start) + 1.0);
  emxEnsureCapacity((emxArray__common *)x2, r2, (int)sizeof(double));
  r1 = (int)((End - Start) + 1.0);
  for (r2 = 0; r2 < r1; r2++) {
    x2->data[r2] = 0.0;
  }

  r2 = x3->size[0] * x3->size[1];
  x3->size[0] = 1;
  x3->size[1] = (int)((End - Start) + 1.0);
  emxEnsureCapacity((emxArray__common *)x3, r2, (int)sizeof(double));
  r1 = (int)((End - Start) + 1.0);
  for (r2 = 0; r2 < r1; r2++) {
    x3->data[r2] = 0.0;
  }

  r2 = z1->size[0] * z1->size[1];
  z1->size[0] = 1;
  z1->size[1] = (int)((End - Start) + 1.0);
  emxEnsureCapacity((emxArray__common *)z1, r2, (int)sizeof(double));
  r1 = (int)((End - Start) + 1.0);
  for (r2 = 0; r2 < r1; r2++) {
    z1->data[r2] = 0.0;
  }

  r2 = z3->size[0] * z3->size[1];
  z3->size[0] = 1;
  z3->size[1] = (int)((End - Start) + 1.0);
  emxEnsureCapacity((emxArray__common *)z3, r2, (int)sizeof(double));
  r1 = (int)((End - Start) + 1.0);
  for (r2 = 0; r2 < r1; r2++) {
    z3->data[r2] = 0.0;
  }

  r2 = theta_1->size[0] * theta_1->size[1];
  theta_1->size[0] = 1;
  theta_1->size[1] = (int)((End - Start) + 1.0);
  emxEnsureCapacity((emxArray__common *)theta_1, r2, (int)sizeof(double));
  r1 = (int)((End - Start) + 1.0);
  for (r2 = 0; r2 < r1; r2++) {
    theta_1->data[r2] = 0.0;
  }

  r2 = theta_3->size[0] * theta_3->size[1];
  theta_3->size[0] = 1;
  theta_3->size[1] = (int)((End - Start) + 1.0);
  emxEnsureCapacity((emxArray__common *)theta_3, r2, (int)sizeof(double));
  r1 = (int)((End - Start) + 1.0);
  for (r2 = 0; r2 < r1; r2++) {
    theta_3->data[r2] = 0.0;
  }

  r2 = (int)((End + 1.0) + (1.0 - (Start + 1.0)));
  for (r1 = 0; r1 < r2; r1++) {
    a22 = (Start + 1.0) + (double)r1;
    x1->data[(int)(a22 - Start) - 1] = x1_0 + 0.5 * lfoot * cos(x[0] * t->data
      [(int)a22 - 1] + x[1]);
    x3->data[(int)(a22 - Start) - 1] = x3_0 - 0.5 * lfoot * cos(x[2] * t->data
      [(int)a22 - 1] + x[3]);
    x2->data[(int)(a22 - Start) - 1] = ((((((((((((((a21 * exp(-sqrt(g / h) *
      t->data[(int)a22 - 1]) + z * exp(sqrt(g / h) * t->data[(int)a22 - 1])) +
      Af1c_prime * cos(x[0] * t->data[(int)a22 - 1] + x[1])) + Af1s_prime * sin
      (x[0] * t->data[(int)a22 - 1] + x[1])) + Af2s_prime * sin(2.0 * x[0] *
      t->data[(int)a22 - 1] + 2.0 * x[1])) + Cf1_prime) + Ag1c_prime * cos(x[2] *
      t->data[(int)a22 - 1] + x[3])) + Ag1s_prime * sin(x[2] * t->data[(int)a22 -
      1] + x[3])) + Ag2s_prime * sin(2.0 * x[2] * t->data[(int)a22 - 1] + 2.0 *
      x[3])) + Cg1_prime) + Aw2_prime * cos(w2 * t->data[(int)a22 - 1] + fai2)) +
      Cw2_prime) + Af1_plus_prime * sin((x[0] + w2) * t->data[(int)a22 - 1] + (x
      [1] + fai2))) + Af1_minus_prime * sin((x[0] - w2) * t->data[(int)a22 - 1] +
      (x[1] - fai2))) + Ag1_plus_prime * sin((x[2] + w2) * t->data[(int)a22 - 1]
      + (x[3] + fai2))) + Ag1_minus_prime * sin((x[2] - w2) * t->data[(int)a22 -
      1] + (x[3] - fai2));
    z1->data[(int)(a22 - Start) - 1] = 0.5 * lfoot * sin(x[0] * t->data[(int)a22
      - 1] + x[1]);
    z3->data[(int)(a22 - Start) - 1] = -0.5 * lfoot * sin(x[2] * t->data[(int)a22
      - 1] + x[3]);
    theta_1->data[(int)(a22 - Start) - 1] = x[0] * t->data[(int)a22 - 1] + x[1];
    theta_3->data[(int)(a22 - Start) - 1] = x[2] * t->data[(int)a22 - 1] + x[3];
  }
}

/* End of code generation (X_DS_Differential_Equa_Solver.c) */
