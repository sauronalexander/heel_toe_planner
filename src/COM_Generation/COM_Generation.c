/*
 * COM_Generation.c
 *
 * Code generation for function 'COM_Generation'
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
#include "heel_toe_planner/COM_Generation/Y_End.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_rtwutil.h"

/* Function Definitions */
void test_COM_Generation()
{
    printf("Testing COM Generation\n");
}

void b_COM_Generation(double totaltime, double tinit, double tend, double tstep,
                    double ratio, double Period, double lstep, double lfoot,
                    double w, double wfoot, double htar, double Mtot, double m1,
                    double m2, double m3, double g, emxArray_real_T * zmp_x, emxArray_real_T * zmp_y,
                    emxArray_real_T * delta_y, emxArray_real_T * coef_x, emxArray_real_T * coef_y,
                    const double x_init_pframe[2], const double x_init_heel_pframe[2],
                    const double x_end_pframe[2], const double x_end_heel_pframe[2],
                    const double theta_iss[3], const double theta_ess[4],
                    emxArray_real_T * zmpUB_x_t, emxArray_real_T * zmpLB_x_t,
                    emxArray_real_T * zmpUB_x, emxArray_real_T * zmpLB_x,
                    emxArray_real_T * zmpUB_y, emxArray_real_T * zmpLB_y,
                    emxArray_real_T * leftgait_x, emxArray_real_T * rightgait_x,
                    emxArray_real_T * leftgait_y, emxArray_real_T * rightgait_y,
                    double z2, double ze, double zi, emxArray_real_T *x_left,
                    emxArray_real_T *x_right, emxArray_real_T *x_trunk,
                    emxArray_real_T *y_trunk, emxArray_real_T *z_left,
                    emxArray_real_T *z_right, emxArray_real_T *theta_left, emxArray_real_T *theta_right)
{
  double x;
  double Sample_step;
  double Sample_init_SS;
  double Nsample;
  double tds;
  double tss;
  double Sample_init;
  double Sample_ss;
  int jy;
  double anew;
  double y;
  double ndbl;
  double cdiff;
  emxArray_real_T *t;
  int i0;
  int nm1d2;
  int jx;
  double O3;
  double B3;
  double w3;
  double C2_X;
  double C1_X;
  double Constant_prime_X;
  double A3_prime;
  double A1_prime;
  double C2_Y;
  double C1_Y;
  double Constant_prime_Y;
  double ita;
  double fai;
  double b_gamma;
  emxArray_real_T *y2;
  emxArray_real_T *x1;
  emxArray_real_T *x2;
  emxArray_real_T *x3;
  emxArray_real_T *z1;
  emxArray_real_T *z3;
  emxArray_real_T *theta1;
  emxArray_real_T *theta3;
  double y3;
  double i;
  int jdelta_y;
  double offset;

  /* General Initialization */
  x = floor(((totaltime - tinit) - tend) / tstep);

  /* Number of steps */
  Sample_step = floor(tstep / Period);
  Sample_init_SS = floor(10.0 * tinit / (10.0 * Period));
  Nsample = (((x + 2.0) - 2.0) * Sample_step + Sample_init_SS) + (floor(tend /
    Period) + 1.0);

  /* Number of samples  */
  tds = ratio * tstep;
  tss = (1.0 - ratio) * tstep;
  Sample_init = Sample_init_SS + floor(tds / Period);
  Sample_ss = floor(tss / Period);
  if (rtIsNaN(Period) || rtIsNaN(totaltime)) {
    jy = 0;
    anew = rtNaN;
    y = totaltime;
  } else if ((Period == 0.0) || ((0.0 < totaltime) && (Period < 0.0)) ||
             ((totaltime < 0.0) && (Period > 0.0))) {
    jy = -1;
    anew = 0.0;
    y = totaltime;
  } else if (rtIsInf(totaltime)) {
    jy = 0;
    anew = rtNaN;
    y = totaltime;
  } else if (rtIsInf(Period)) {
    jy = 0;
    anew = 0.0;
    y = totaltime;
  } else {
    anew = 0.0;
    ndbl = floor(totaltime / Period + 0.5);
    y = ndbl * Period;
    if (Period > 0.0) {
      cdiff = y - totaltime;
    } else {
      cdiff = totaltime - y;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * fabs(totaltime)) {
      ndbl++;
      y = totaltime;
    } else if (cdiff > 0.0) {
      y = (ndbl - 1.0) * Period;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      jy = (int)ndbl - 1;
    } else {
      jy = -1;
    }
  }

  emxInit_real_T(&t, 2);
  i0 = t->size[0] * t->size[1];
  t->size[0] = 1;
  t->size[1] = jy + 1;
  emxEnsureCapacity((emxArray__common *)t, i0, (int)sizeof(double));
  if (jy + 1 > 0) {
    t->data[0] = anew;
    if (jy + 1 > 1) {
      t->data[jy] = y;
      nm1d2 = jy / 2;
      for (jx = 1; jx < nm1d2; jx++) {
        ndbl = (double)jx * Period;
        t->data[jx] = anew + ndbl;
        t->data[jy - jx] = y - ndbl;
      }

      if (nm1d2 << 1 == jy) {
        t->data[nm1d2] = (anew + y) / 2.0;
      } else {
        ndbl = (double)nm1d2 * Period;
        t->data[nm1d2] = anew + ndbl;
        t->data[nm1d2 + 1] = y - ndbl;
      }
    }
  }

  double theta_start = (theta_iss[0]+theta_iss[1]);
  double theta_end = (theta_ess[0]+theta_ess[1]-theta_ess[2]);

  /* initiallization */
  i0 = x_left->size[0] * x_left->size[1];
  x_left->size[0] = 1;
  x_left->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)x_left, i0, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i0 = 0; i0 < nm1d2; i0++) {
    x_left->data[i0] = 0.0;
  }

  i0 = x_right->size[0] * x_right->size[1];
  x_right->size[0] = 1;
  x_right->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)x_right, i0, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i0 = 0; i0 < nm1d2; i0++) {
    x_right->data[i0] = 0.0;
  }

  i0 = x_trunk->size[0] * x_trunk->size[1];
  x_trunk->size[0] = 1;
  x_trunk->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)x_trunk, i0, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i0 = 0; i0 < nm1d2; i0++) {
    x_trunk->data[i0] = 0.0;
  }

  i0 = y_trunk->size[0] * y_trunk->size[1];
  y_trunk->size[0] = 1;
  y_trunk->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)y_trunk, i0, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i0 = 0; i0 < nm1d2; i0++) {
    y_trunk->data[i0] = 0.0;
  }

  i0 = z_left->size[0] * z_left->size[1];
  z_left->size[0] = 1;
  z_left->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)z_left, i0, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i0 = 0; i0 < nm1d2; i0++) {
    z_left->data[i0] = 0.0;
  }

  i0 = z_right->size[0] * z_right->size[1];
  z_right->size[0] = 1;
  z_right->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)z_right, i0, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i0 = 0; i0 < nm1d2; i0++) {
    z_right->data[i0] = 0.0;
  }

  i0 = theta_left->size[0] * theta_left->size[1];
  theta_left->size[0] = 1;
  theta_left->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)theta_left, i0, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i0 = 0; i0 < nm1d2; i0++) {
    theta_left->data[i0] = 0.0;
  }

  i0 = theta_right->size[0] * theta_right->size[1];
  theta_right->size[0] = 1;
  theta_right->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)theta_right, i0, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i0 = 0; i0 < nm1d2; i0++) {
    theta_right->data[i0] = 0.0;
  }

  /* Sample_init */
  /* X_Component */
  cdiff = zmpLB_x_t->data[0] + 0.5 * lfoot;
  ndbl = zmpUB_x_t->data[(int)(Sample_init_SS + 1.0) - 1] + 0.5 * lfoot * cos
    ((theta_ess[0] + theta_ess[1]) - theta_ess[2]);
  O3 = 0.5 * (cdiff + ndbl);
  B3 = 0.5 * (ndbl - cdiff);
  w3 = 3.1415926535897931 / tinit;
  X_SS_Differential_Equa_Solver(g, z2, -Mtot * g * coef_x->data[0] / (m2 * z2),
    coef_x->data[1], coef_x->data[2], m3 * g * B3 / (m2 * z2) + m3 * x_end_pframe
    [1] * (w3 * w3) / (m2 * z2), w3, -b_pi, (-Mtot * coef_x->data[3] / m2 + m3 * O3 / m2)
    + m1 * cdiff / m2, htar * lfoot, leftgait_x->data[(int)(Sample_init_SS + 1.0)
    - 1] - x_end_heel_pframe[0], tinit, &A1_prime, &A3_prime, &Constant_prime_X,
    &C1_X, &C2_X);

  /* Y_Component */
  Y_SS_Differential_Equa_Solver(g, z2, -Mtot * g * delta_y->data[0] / (m2 * z2),
    -Mtot * g * delta_y->data[1] / (m2 * z2), -Mtot * g * (delta_y->data[2] +
    coef_y->data[2]) / (m2 * z2), -Mtot * g * (delta_y->data[3] + coef_y->data[1]) /
                                (m2 * z2), -Mtot * g * (delta_y->data[4] +
    coef_y->data[0]) / (m2 * z2), m1 * g * (w / 2.0) / (m2 * z2) + m3 * g * (-w /
    2.0) / (m2 * z2), w / 8.0, w / 8.0, tinit, &anew, &y, &b_gamma, &fai, &ita,
    &Constant_prime_Y, &C1_Y, &C2_Y);
  for (nm1d2 = 0; nm1d2 < (int)(Sample_init_SS + 1.0); nm1d2++) {
    x_left->data[nm1d2] = B3 * cos(w3 * t->data[nm1d2] + -3.1415926535897931) +
      O3;
    x_right->data[nm1d2] = zmpLB_x_t->data[0] + 0.5 * lfoot;
    x_trunk->data[nm1d2] = (((C1_X * exp(-sqrt(g / z2) * t->data[nm1d2]) + C2_X *
      exp(sqrt(g / z2) * t->data[nm1d2])) + A1_prime * cos(coef_x->data[1] *
      t->data[nm1d2] + coef_x->data[2])) + A3_prime * cos(w3 * t->data[nm1d2] +
      -3.1415926535897931)) + Constant_prime_X;
    y_trunk->data[nm1d2] = ((((((C1_Y * exp(-sqrt(g / z2) * t->data[nm1d2]) +
      C2_Y * exp(sqrt(g / z2) * t->data[nm1d2])) + anew * rt_powd_snf(t->
      data[nm1d2], 4.0)) + y * rt_powd_snf(t->data[nm1d2], 3.0)) + b_gamma *
      (t->data[nm1d2] * t->data[nm1d2])) + fai * t->data[nm1d2]) + ita) +
      Constant_prime_Y;
    z_left->data[nm1d2] = zi;
    z_right->data[nm1d2] = 0.0;
    theta_left->data[nm1d2] = theta_start + t->data[nm1d2]*(theta_end-theta_start) / tinit;
    theta_right->data[nm1d2] = 0.0;

    /* y_left(i) = C1_Y*exp(-sqrt(g/ze)*t(i)) +  C2_Y*exp(sqrt(g/ze)*t(i)) + ... */
    /*     alpha*t(i)^4 + beta*t(i)^3 + gamma*t(i)^2 + fai*t(i) + ita + Constant_prime_Y; */
  }

  emxInit_real_T(&y2, 2);
  emxInit_real_T(&x1, 2);
  emxInit_real_T(&x2, 2);
  emxInit_real_T(&x3, 2);
  emxInit_real_T(&z1, 2);
  emxInit_real_T(&z3, 2);
  emxInit_real_T(&theta1, 2);
  emxInit_real_T(&theta3, 2);

  /* Double Support of Sample_init */
  /* Left foot in front */
  X_DS_Differential_Equa_Solver(Sample_init_SS, Sample_init, t, g, z2, lfoot,
    tds, theta_ess, theta_iss, m1, m2, m3, ze, coef_x->data[4], coef_x->data[5],
    coef_x->data[6], coef_x->data[7], tinit, leftgait_x->data[(int)(Sample_init_SS
    + 1.0) - 1], rightgait_x->data[(int)(Sample_init_SS + 1.0) - 1] + lfoot,
    leftgait_x->data[(int)(Sample_init_SS + 1.0) - 1] - x_end_heel_pframe[0],
    rightgait_x->data[(int)(Sample_init_SS + 1.0) - 1] - x_init_heel_pframe[0],
    x1, x2, x3, z1, z3, theta1, theta3);
  cdiff = w / 4.0;
  y3 = -w / 4.0;
  b_Y_DS_Differential_Equa_Solver(Sample_init_SS, Sample_init, t, g, z2, lfoot,
    tds, theta_ess, theta_iss, m1, m2, m3, cdiff, y3, coef_y->data[3],
    coef_y->data[4], coef_y->data[5], tinit, w / 8.0, -w / 8.0, y2);
  i0 = (int)((Sample_init + 1.0) + (1.0 - (Sample_init_SS + 1.0)));
  for (nm1d2 = 0; nm1d2 < i0; nm1d2++) {
    i = (Sample_init_SS + 1.0) + (double)nm1d2;
    x_left->data[(int)i - 1] = x1->data[(int)(i - Sample_init_SS) - 1];
    x_right->data[(int)i - 1] = x3->data[(int)(i - Sample_init_SS) - 1];
    x_trunk->data[(int)i - 1] = x2->data[(int)(i - Sample_init_SS) - 1];
    y_trunk->data[(int)i - 1] = y2->data[(int)(i - Sample_init_SS) - 1];
    z_left->data[(int)i - 1] = z1->data[(int)(i - Sample_init_SS) - 1];
    z_right->data[(int)i - 1] = z3->data[(int)(i - Sample_init_SS) - 1];
    theta_left->data[(int)i - 1] = theta1->data[(int)(i - Sample_init_SS) - 1];
    theta_right->data[(int)i - 1] = theta3->data[(int)(i - Sample_init_SS) - 1];
  }
  /* Normal Steps */
  Sample_init_SS = 2.0;
  jx = 9;

  /* for coefx index */
  jy = 7;

  /* for coefy index */
  jdelta_y = 6;

  /* for deltay index */
  while (Sample_init_SS < (x + 2.0) - 1.0) {
    offset = (Sample_init_SS - 2.0) * Sample_step + Sample_init;

    /* Left foot Standing */
    /* X_Component */
    ndbl = zmpUB_x_t->data[(int)((offset + Sample_ss) + 1.0) - 1] + 0.5 * lfoot *
      cos((theta_ess[0] + theta_ess[1]) - theta_ess[2]);
    cdiff = rightgait_x->data[(int)((offset + Sample_ss) + 1.0) - 1] -
      x_end_heel_pframe[0];
    O3 = 0.5 * (x_right->data[(int)(offset + 1.0) - 1] + ndbl);
    B3 = 0.5 * (ndbl - x_right->data[(int)(offset + 1.0) - 1]);
    w3 = 3.1415926535897931 / tss;
    X_SS_Differential_Equa_Solver(g, z2, -Mtot * g * coef_x->data[jx - 1] / (m2 *
      z2), coef_x->data[jx], coef_x->data[jx + 1], m3 * g * B3 / (m2 * z2) + m3 *
      x_end_pframe[1] * (w3 * w3) / (m2 * z2), w3, -b_pi, (-Mtot * coef_x->data[jx + 2] /
      m2 + m3 * O3 / m2) + m1 * x_right->data[(int)(offset + 1.0) - 1] / m2,
      x_trunk->data[(int)(offset + 1.0) - 1], cdiff, tss, &A1_prime, &A3_prime,
      &Constant_prime_X, &C1_X, &C2_X);

    /* Y_Component */
    Y_SS_Differential_Equa_Solver(g, z2, -Mtot * g * delta_y->data[jdelta_y - 1] /
                                  (m2 * z2), -Mtot * g * delta_y->data[jdelta_y] /
                                  (m2 * z2), -Mtot * g * (delta_y->data[jdelta_y
      + 1] + coef_y->data[jy + 1]) / (m2 * z2), -Mtot * g *
      (delta_y->data[jdelta_y + 2] + coef_y->data[jy]) / (m2 * z2), -Mtot * g *
      (delta_y->data[jdelta_y + 3] + coef_y->data[jy - 1]) / (m2 * z2), m1 * g * (
      -w / 2.0) / (m2 * z2) + m3 * g * (w / 2.0) / (m2 * z2), -w / 8.0, -w / 8.0,
      tss, &anew, &y, &b_gamma, &fai, &ita, &Constant_prime_Y, &C1_Y, &C2_Y);
    i0 = (int)(((offset + Sample_ss) + 1.0) + (1.0 - (offset + 1.0)));
    for (nm1d2 = 0; nm1d2 < i0; nm1d2++) {
      i = (offset + 1.0) + (double)nm1d2;
      x_right->data[(int)i - 1] = B3 * cos(w3 * (t->data[(int)i - 1] - t->data
        [(int)(offset + 1.0) - 1]) + -3.1415926535897931) + O3;
      x_left->data[(int)i - 1] = zmpLB_x_t->data[(int)(offset + 1.0) - 1] + 0.5 *
        lfoot;
      x_trunk->data[(int)i - 1] = (((C1_X * exp(-sqrt(g / z2) * (t->data[(int)i
        - 1] - t->data[(int)(offset + 1.0) - 1])) + C2_X * exp(sqrt(g / z2) *
        (t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1]))) + A1_prime *
        cos(coef_x->data[jx] * (t->data[(int)i - 1] - t->data[(int)(offset + 1.0)
        - 1]) + coef_x->data[jx + 1])) + A3_prime * cos(w3 * (t->data[(int)i - 1]
        - t->data[(int)(offset + 1.0) - 1]) + -3.1415926535897931)) +
        Constant_prime_X;
      ndbl = t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1];
      y_trunk->data[(int)i - 1] = ((((((C1_Y * exp(-sqrt(g / z2) * (t->data[(int)
        i - 1] - t->data[(int)(offset + 1.0) - 1])) + C2_Y * exp(sqrt(g / z2) *
        (t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1]))) + anew *
        rt_powd_snf(t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1], 4.0))
        + y * rt_powd_snf(t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1],
                          3.0)) + b_gamma * (ndbl * ndbl)) + fai * (t->data[(int)
        i - 1] - t->data[(int)(offset + 1.0) - 1])) + ita) + Constant_prime_Y;
      z_left->data[(int)i - 1] = 0.0;
      z_right->data[(int)i - 1] = zi;
      theta_left->data[(int)i - 1] = 0.0;
      theta_right->data[(int)i - 1] = theta_start + (t->data[(int)i-1] - t->data[(int)(offset + 1.0)-1])
                                * (theta_end - theta_start)/tss;
    }

    /* Double Support of Sample_init */
    /* Left foot in front */
    ndbl = x_trunk->data[(int)((offset + Sample_ss) + 1.0) - 1];
    X_DS_Differential_Equa_Solver(offset + Sample_ss, offset + Sample_step, t, g,
      z2, lfoot, tds, theta_ess, theta_iss, m1, m2, m3, ze, coef_x->data[jx + 3],
      coef_x->data[jx + 4], coef_x->data[jx + 5], coef_x->data[jx + 6], t->data
      [(int)((offset + Sample_ss) + 1.0) - 1], rightgait_x->data[(int)((offset +
      Sample_ss) + 1.0) - 1], leftgait_x->data[(int)((offset + Sample_ss) + 1.0)
      - 1] + lfoot, ndbl, cdiff, x1, x2, x3, z1, z3, theta1, theta3);
    b_Y_DS_Differential_Equa_Solver(offset + Sample_ss, offset + Sample_step, t, g,
      z2, lfoot, tds, theta_ess, theta_iss, m1, m2, m3, -w / 2.0, w / 2.0,
      coef_y->data[jy + 2], coef_y->data[jy + 3], coef_y->data[jy + 4], t->data
      [(int)((offset + Sample_ss) + 1.0) - 1], -w / 8.0, w / 8.0, y2);
    ndbl = (offset + Sample_ss) + 1.0;
    i0 = (int)(((offset + Sample_step) + 1.0) + (1.0 - ndbl));
    for (nm1d2 = 0; nm1d2 < i0; nm1d2++) {
      i = ndbl + (double)nm1d2;
      x_left->data[(int)i - 1] = x3->data[(int)((i - offset) - Sample_ss) - 1];
      x_right->data[(int)i - 1] = x1->data[(int)((i - offset) - Sample_ss) - 1];
      x_trunk->data[(int)i - 1] = x2->data[(int)((i - offset) - Sample_ss) - 1];
      y_trunk->data[(int)i - 1] = y2->data[(int)((i - offset) - Sample_ss) - 1];
      z_left->data[(int)i - 1] = z3->data[(int)((i - offset) - Sample_ss) - 1];
      z_right->data[(int)i - 1] = z1->data[(int)((i - offset) - Sample_ss) - 1];
      theta_left->data[(int)i - 1] = theta3->data[(int)((i - offset) - Sample_ss) - 1];
      theta_right->data[(int)i - 1] = theta1->data[(int)((i - offset) - Sample_ss) - 1];
    }

    jx += 8;
    offset = (Sample_init_SS - 1.0) * Sample_step + Sample_init;

    /* Right foot Standing */
    /* X_Component */
    ndbl = zmpUB_x_t->data[(int)((offset + Sample_ss) + 1.0) - 1] + 0.5 * lfoot *
      cos((theta_ess[0] + theta_ess[1]) - theta_ess[2]);
    cdiff = leftgait_x->data[(int)((offset + Sample_ss) + 1.0) - 1] -
      x_end_heel_pframe[0];
    O3 = 0.5 * (x_left->data[(int)(offset + 1.0) - 1] + ndbl);
    B3 = 0.5 * (ndbl - x_left->data[(int)(offset + 1.0) - 1]);
    X_SS_Differential_Equa_Solver(g, z2, -Mtot * g * coef_x->data[jx - 1] / (m2 *
      z2), coef_x->data[jx], coef_x->data[jx + 1], m3 * g * B3 / (m2 * z2) + m3 *
      x_end_pframe[1] * (w3 * w3) / (m2 * z2), w3, -b_pi, (-Mtot * coef_x->data[jx + 2] /
      m2 + m3 * O3 / m2) + m1 * x_left->data[(int)(offset + 1.0) - 1] / m2,
      x_trunk->data[(int)(offset + 1.0) - 1], cdiff, tss, &A1_prime, &A3_prime,
      &Constant_prime_X, &C1_X, &C2_X);

    /* Y_Component */
    Y_SS_Differential_Equa_Solver(g, z2, -Mtot * g * delta_y->data[jdelta_y + 4] /
                                  (m2 * z2), -Mtot * g * delta_y->data[jdelta_y +
      5] / (m2 * z2), -Mtot * g * (delta_y->data[jdelta_y + 6] + coef_y->data[jy +
      7]) / (m2 * z2), -Mtot * g * (delta_y->data[jdelta_y + 7] + coef_y->data[jy
      + 6]) / (m2 * z2), -Mtot * g * (delta_y->data[jdelta_y + 8] +
      coef_y->data[jy + 5]) / (m2 * z2), m1 * g * (-w / 2.0) / (m2 * z2) + m3 * g
      * (w / 2.0) / (m2 * z2), w / 8.0, w / 8.0, tss, &anew, &y, &b_gamma, &fai,
      &ita, &Constant_prime_Y, &C1_Y, &C2_Y);
    i0 = (int)(((offset + Sample_ss) + 1.0) + (1.0 - (offset + 1.0)));
    for (nm1d2 = 0; nm1d2 < i0; nm1d2++) {
      i = (offset + 1.0) + (double)nm1d2;
      x_left->data[(int)i - 1] = B3 * cos(w3 * (t->data[(int)i - 1] - t->data
        [(int)(offset + 1.0) - 1]) + -3.1415926535897931) + O3;
      x_right->data[(int)i - 1] = zmpLB_x_t->data[(int)(offset + 1.0) - 1] + 0.5 *
        lfoot;
      x_trunk->data[(int)i - 1] = (((C1_X * exp(-sqrt(g / z2) * (t->data[(int)i
        - 1] - t->data[(int)(offset + 1.0) - 1])) + C2_X * exp(sqrt(g / z2) *
        (t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1]))) + A1_prime *
        cos(coef_x->data[jx] * (t->data[(int)i - 1] - t->data[(int)(offset + 1.0)
        - 1]) + coef_x->data[jx + 1])) + A3_prime * cos(w3 * (t->data[(int)i - 1]
        - t->data[(int)(offset + 1.0) - 1]) + -3.1415926535897931)) +
        Constant_prime_X;
      ndbl = t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1];
      y_trunk->data[(int)i - 1] = ((((((C1_Y * exp(-sqrt(g / z2) * (t->data[(int)
        i - 1] - t->data[(int)(offset + 1.0) - 1])) + C2_Y * exp(sqrt(g / z2) *
        (t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1]))) + anew *
        rt_powd_snf(t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1], 4.0))
        + y * rt_powd_snf(t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1],
                          3.0)) + b_gamma * (ndbl * ndbl)) + fai * (t->data[(int)
        i - 1] - t->data[(int)(offset + 1.0) - 1])) + ita) + Constant_prime_Y;
      z_left->data[(int)i - 1] = zi;
      z_right->data[(int)i - 1] = 0.0;
      theta_left->data[(int)i - 1] = theta_start + (t->data[(int)i-1] - t->data[(int)(offset + 1.0)-1])
                                 * (theta_end - theta_start)/tss;
      theta_right->data[(int)i - 1] = 0.0;
    }

    /* Double Support of Sample_init */
    /* Left foot in front */
    ndbl = x_trunk->data[(int)((offset + Sample_ss) + 1.0) - 1];
    X_DS_Differential_Equa_Solver(offset + Sample_ss, offset + Sample_step, t,
      g, z2, lfoot, tds, theta_ess, theta_iss, m1, m2, m3, ze, coef_x->data[jx +
      3], coef_x->data[jx + 4], coef_x->data[jx + 5], coef_x->data[jx + 6], t->
      data[(int)((offset + Sample_ss) + 1.0) - 1], leftgait_x->data[(int)((offset
      + Sample_ss) + 1.0) - 1], rightgait_x->data[(int)((offset + Sample_ss) +
      1.0) - 1] + lfoot, ndbl, cdiff, x1, x2, x3, z1, z3, theta1, theta3);
    cdiff = -w / 2.0;
    y3 = w / 2.0;
    b_Y_DS_Differential_Equa_Solver(offset + Sample_ss, offset + Sample_step, t, g,
      z2, lfoot, tds, theta_ess, theta_iss, m1, m2, m3, cdiff, y3,
      coef_y->data[jy + 8], coef_y->data[jy + 9], coef_y->data[jy + 10], t->data
      [(int)((offset + Sample_ss) + 1.0) - 1], w / 8.0, -w / 8.0, y2);
    ndbl = (offset + Sample_ss) + 1.0;
    i0 = (int)(((offset + Sample_step) + 1.0) + (1.0 - ndbl));
    for (nm1d2 = 0; nm1d2 < i0; nm1d2++) {
      i = ndbl + (double)nm1d2;
      x_left->data[(int)i - 1] = x1->data[(int)((i - offset) - Sample_ss) - 1];
      x_right->data[(int)i - 1] = x3->data[(int)((i - offset) - Sample_ss) - 1];
      x_trunk->data[(int)i - 1] = x2->data[(int)((i - offset) - Sample_ss) - 1];
      y_trunk->data[(int)i - 1] = y2->data[(int)((i - offset) - Sample_ss) - 1];
      z_left->data[(int)i - 1] = z1->data[(int)((i - offset) - Sample_ss) - 1];
      z_right->data[(int)i - 1] = z3->data[(int)((i - offset) - Sample_ss) - 1];
      theta_left->data[(int)i - 1] = theta1->data[(int)((i - offset) - Sample_ss) - 1];
      theta_right->data[(int)i - 1] = theta3->data[(int)((i - offset) - Sample_ss) - 1];
    }

    jx += 8;
    jy += 12;
    jdelta_y += 10;
    Sample_init_SS += 2.0;
  }

  emxFree_real_T(&z3);
  emxFree_real_T(&z1);
  emxFree_real_T(&x3);
  emxFree_real_T(&x2);
  emxFree_real_T(&x1);
  emxFree_real_T(&y2);
  /* End Phase */
  /* X Component */
  offset = (Sample_init_SS - 2.0) * Sample_step + Sample_init;
  ndbl = rightgait_x->data[(int)Nsample - 1] + 0.5 * lfoot;
  O3 = 0.5 * (x_right->data[(int)(offset + 1.0) - 1] + ndbl);
  B3 = 0.5 * (ndbl - x_right->data[(int)(offset + 1.0) - 1]);
  w3 = 3.1415926535897931 / (tend - tds);
  X_SS_Differential_Equa_Solver(g, z2, -Mtot * g * coef_x->data[jx - 1] / (m2 *
    z2), coef_x->data[jx], coef_x->data[jx + 1], m3 * g * B3 / (m2 * z2) + m3 *
    x_end_pframe[1] * (w3 * w3) / (m2 * z2), w3, -b_pi, (-Mtot * coef_x->data[jx + 2] /
    m2 + m3 * O3 / m2) + m1 * x_right->data[(int)(offset + 1.0) - 1] / m2,
    x_trunk->data[(int)(offset + 1.0) - 1], leftgait_x->data[(int)(offset + 1.0)
    - 1] + htar * lfoot, tinit, &A1_prime, &A3_prime, &Constant_prime_X, &C1_X,
    &C2_X);

  /* Y Component */
  Y_End(g, z2, -Mtot * g * delta_y->data[jdelta_y - 1] / (m2 * z2), -Mtot * g *
        delta_y->data[jdelta_y] / (m2 * z2), -Mtot * g * delta_y->data[jdelta_y +
        1] / (m2 * z2), -Mtot * g * delta_y->data[jdelta_y + 2] / (m2 * z2),
        -Mtot * g * delta_y->data[jdelta_y + 3] / (m2 * z2), m1 * g * cdiff / (m2
         * z2) + m3 * g * y3 / (m2 * z2), -w / 8.0, 0.0, tend - tds, &anew, &y,
        &b_gamma, &fai, &ita, &Constant_prime_Y, &C1_Y, &C2_Y);
  i0 = (int)(Nsample + (1.0 - (offset + 1.0)));
  for (nm1d2 = 0; nm1d2 < i0; nm1d2++) {
    i = (offset + 1.0) + (double)nm1d2;
    x_right->data[(int)i - 1] = B3 * cos(w3 * (t->data[(int)i - 1] - t->data
      [(int)(offset + 1.0) - 1]) + -3.1415926535897931) + O3;
    x_left->data[(int)i - 1] = zmpLB_x_t->data[(int)(offset + 1.0) - 1] + 0.5 *
      lfoot;
    x_trunk->data[(int)i - 1] = (((C1_X * exp(-sqrt(g / z2) * (t->data[(int)i -
      1] - t->data[(int)(offset + 1.0) - 1])) + C2_X * exp(sqrt(g / z2) *
      (t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1]))) + A1_prime *
      cos(coef_x->data[jx] * (t->data[(int)i - 1] - t->data[(int)(offset + 1.0) -
      1]) + coef_x->data[jx + 1])) + A3_prime * cos(w3 * (t->data[(int)i - 1] -
      t->data[(int)(offset + 1.0) - 1]) + -3.1415926535897931)) +
      Constant_prime_X;
    ndbl = t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1];
    y_trunk->data[(int)i - 1] = ((((((C1_Y * exp(-sqrt(g / z2) * (t->data[(int)i
      - 1] - t->data[(int)(offset + 1.0) - 1])) + C2_Y * exp(sqrt(g / z2) *
      (t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1]))) + anew *
      rt_powd_snf(t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1], 4.0))
      + y * rt_powd_snf(t->data[(int)i - 1] - t->data[(int)(offset + 1.0) - 1],
                        3.0)) + b_gamma * (ndbl * ndbl)) + fai * (t->data[(int)i
      - 1] - t->data[(int)(offset + 1.0) - 1])) + ita) + Constant_prime_Y;
    z_left->data[(int)i - 1] = 0.0;
    z_right->data[(int)i - 1] = zi;
    theta_left->data[(int)i - 1] = 0.0;
    theta_right->data[(int)i - 1] = theta_start + (t->data[(int)i-1] - t->data[(int)(offset + 1.0)-1])
            * (theta_end - theta_start)/(tend-tds);
  }

  for(int i=0; i<theta_left->size[1]; i++)
  {
      theta_left->data[i] = -1.0*theta_left->data[i];
      theta_right->data[i] = -1.0*theta_right->data[i];
  }

  emxFree_real_T(&t);

  /* { */
  /* figure(5); */
  /* subplot(3, 1, 1); */
  /* plot(t,x_right); */
  /* hold on; */
  /* plot(t, x_left, 'g'); */
  /* plot(t, x_trunk, 'r'); */
  /* hold off; */
  /* ylabel('x(m)'); */
  /* xlabel('t(s)'); */
  /* legend('Right Foot', 'Left Foot', 'Trunk'); */
  /* title('COM Generation--X'); */
  /*  */
  /* subplot(3, 1, 2); */
  /* plot(t, y_trunk, 'r'); */
  /* ylabel('x(m)'); */
  /* xlabel('t(s)'); */
  /* title('COM Generation of Trunk--Y'); */
  /*  */
  /* subplot(3, 1, 3); */
  /* plot(t, z_left, 'g'); */
  /* hold on; */
  /* plot(t, z_right, 'r'); */
  /* hold off; */
  /* ylabel('x(m)'); */
  /* xlabel('t(s)'); */
  /* legend('Right Foot', 'Left Foot'); */
  /* title('COM Generation--Z'); */
  /* } */
}

/* End of code generation (COM_Generation.c) */
