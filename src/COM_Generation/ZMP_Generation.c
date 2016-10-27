/*
 * ZMP_Generation.c
 *
 * Code generation for function 'ZMP_Generation'
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
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_emxutil.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_rtwutil.h"
#include <stdio.h>

/* Function Definitions */
void test_ZMP_Generation()
{
    printf("Testing ZMP Generation\n");
}

void ZMP_Generation(emxArray_real_T * zmpLB_x_t, emxArray_real_T * zmpUB_x_t, double totaltime,
                    double tinit, double tend, double tstep, double ratio, double Period, double lstep,
                    double lfoot, double w, double wfoot, double htar, emxArray_real_T * zmp_x,
                    emxArray_real_T * zmp_y, emxArray_real_T * delta_y, emxArray_real_T * coef_x,
                    emxArray_real_T * coef_y)
{
  double jy;
  double x;
  double N;
  double Sample_step;
  double Sample_init_SS;
  double Nsample;
  double tds;
  double tss;
  double Sample_init;
  double Sample_ss;
  int n;
  double jx;
  double ndbl;
  double cdiff;
  emxArray_real_T *t;
  int i12;
  int nm1d2;
  int k;
  double delta12;
  double delta23;
  double temp_x[8];
  double dv11[6];
  double zmp_y_prime0;
  double dv12[5];
  (void)lstep;

  /* %ZMP Generation */
  /* input: totaltime, tinit, tend, tstep, ratio, Period(for sample), lstep, lfoot, w(hipwidth), wfoot, htar; */
  /* zmpUB_x_t, zmpLB_x_t, zmpUB_x, zmpLB_x, zmpUB_y, zmpLB_y, leftgait_x, leftgait_y, rightgait_x, rightgait_y; */
  /* output: zmp_x, zmp_y, delta_y; */
  /*  */
  /* x_component using Cosine_Cosine_Interpolation: */
  /* Output: [coef_x] = [A1, w1, fai1, O1, A2, w2, fai2, O2]'; */
  /* Input: Cosine_Cosine_Interpolation(delta01, delta12, delta23, zmp_x_0, zmp_x_1, zmp_x_3) */
  /*  */
  /* y_component using Quadratic_Cosine_Interpolation: */
  /* Output: [coef_y] = [b0, b1, b2, B0, w3, fai3]'; */
  /*  */
  /* y_correction function */
  /* This function only apply to single support state */
  /* delta(t) = Ax^4+Bx^3+Cx^2+Dx+E; */
  /* Output: [delta_y] =[A, B, C, D, E]'; */
  /*  */
  /* Input: Quadratic_Cosine_Interpolation(delta01, delta12, delta23, zmp_y_0, zmp_y_1, zmp_y_3, 0) */
  /* General Initialization */
  jy = ((totaltime - tinit) - tend) / tstep;
  x = floor(jy);
  N = floor(jy) + 2.0;

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
    n = 0;
    jx = rtNaN;
    jy = totaltime;
  } else if ((Period == 0.0) || ((0.0 < totaltime) && (Period < 0.0)) ||
             ((totaltime < 0.0) && (Period > 0.0))) {
    n = -1;
    jx = 0.0;
    jy = totaltime;
  } else if (rtIsInf(totaltime)) {
    n = 0;
    jx = rtNaN;
    jy = totaltime;
  } else if (rtIsInf(Period)) {
    n = 0;
    jx = 0.0;
    jy = totaltime;
  } else {
    jx = 0.0;
    ndbl = floor(totaltime / Period + 0.5);
    jy = ndbl * Period;
    if (Period > 0.0) {
      cdiff = jy - totaltime;
    } else {
      cdiff = totaltime - jy;
    }

    if (fabs(cdiff) < 4.4408920985006262E-16 * fabs(totaltime)) {
      ndbl++;
      jy = totaltime;
    } else if (cdiff > 0.0) {
      jy = (ndbl - 1.0) * Period;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl - 1;
    } else {
      n = -1;
    }
  }

  emxInit_real_T(&t, 2);
  i12 = t->size[0] * t->size[1];
  t->size[0] = 1;
  t->size[1] = n + 1;
  emxEnsureCapacity((emxArray__common *)t, i12, (int)sizeof(double));
  if (n + 1 > 0) {
    t->data[0] = jx;
    if (n + 1 > 1) {
      t->data[n] = jy;
      nm1d2 = n / 2;
      for (k = 1; k < nm1d2; k++) {
        ndbl = (double)k * Period;
        t->data[k] = jx + ndbl;
        t->data[n - k] = jy - ndbl;
      }

      if (nm1d2 << 1 == n) {
        t->data[nm1d2] = (jx + jy) / 2.0;
      } else {
        ndbl = (double)nm1d2 * Period;
        t->data[nm1d2] = jx + ndbl;
        t->data[nm1d2 + 1] = jy - ndbl;
      }
    }
  }

  /* initiallization */
  i12 = zmp_x->size[0] * zmp_x->size[1];
  zmp_x->size[0] = 1;
  zmp_x->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)zmp_x, i12, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i12 = 0; i12 < nm1d2; i12++) {
    zmp_x->data[i12] = 0.0;
  }

  i12 = zmp_y->size[0] * zmp_y->size[1];
  zmp_y->size[0] = 1;
  zmp_y->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)zmp_y, i12, (int)sizeof(double));
  nm1d2 = (int)Nsample;
  for (i12 = 0; i12 < nm1d2; i12++) {
    zmp_y->data[i12] = 0.0;
  }

  jy = 8.0 * (N - 1.0);
  i12 = coef_x->size[0] * coef_x->size[1];
  coef_x->size[0] = 1;
  coef_x->size[1] = (int)(jy + 4.0);
  emxEnsureCapacity((emxArray__common *)coef_x, i12, (int)sizeof(double));
  nm1d2 = (int)(jy + 4.0);
  for (i12 = 0; i12 < nm1d2; i12++) {
    coef_x->data[i12] = 0.0;
  }

  jy = 6.0 * (N - 1.0);
  i12 = coef_y->size[0] * coef_y->size[1];
  coef_y->size[0] = 1;
  coef_y->size[1] = (int)(jy + 4.0);
  emxEnsureCapacity((emxArray__common *)coef_y, i12, (int)sizeof(double));
  nm1d2 = (int)(jy + 4.0);
  for (i12 = 0; i12 < nm1d2; i12++) {
    coef_y->data[i12] = 0.0;
  }

  jy = 5.0 * (x + 2.0);
  i12 = delta_y->size[0] * delta_y->size[1];
  delta_y->size[0] = 1;
  delta_y->size[1] = (int)jy;
  emxEnsureCapacity((emxArray__common *)delta_y, i12, (int)sizeof(double));
  nm1d2 = (int)jy;
  for (i12 = 0; i12 < nm1d2; i12++) {
    delta_y->data[i12] = 0.0;
  }

  /* Sample_init */
  delta12 = 0.5 * tds;
  delta23 = 0.5 * tds;
  Cosine_Cosine_Interpolation(tinit, delta12, delta23, zmpLB_x_t->data[0] + htar *
    lfoot, zmpUB_x_t->data[(int)Sample_init_SS - 1], zmpUB_x_t->data[(int)
    Sample_init - 1], temp_x);
  for (i12 = 0; i12 < 8; i12++) {
    coef_x->data[i12] = temp_x[i12];
  }

  Quadratic_Cosine_Interpolation(tinit, delta12, delta23, w / 2.0, w / 2.0 -
    wfoot / 2.0, -w / 2.0 + wfoot / 2.0, 0.0, dv11);
  for (i12 = 0; i12 < 6; i12++) {
    coef_y->data[i12] = dv11[i12];
  }

  zmp_y_prime0 = -coef_y->data[3] * coef_y->data[4] * sin(coef_y->data[4] *
    (tinit + tds) + coef_y->data[5]);
  for (i12 = 0; i12 < 5; i12++) {
    delta_y->data[i12] = 0.0;
  }

  for (nm1d2 = 0; nm1d2 < (int)Sample_init_SS; nm1d2++) {
    zmp_x->data[nm1d2] = coef_x->data[0] * cos(coef_x->data[1] * t->data[nm1d2]
      + coef_x->data[2]) + coef_x->data[3];
    zmp_y->data[nm1d2] = (coef_y->data[0] + coef_y->data[1] * t->data[nm1d2]) +
      coef_y->data[2] * (t->data[nm1d2] * t->data[nm1d2]);
  }

  i12 = (int)((float)Sample_init + (1.0F - ((float)Sample_init_SS + 1.0F)));
  for (nm1d2 = 0; nm1d2 < i12; nm1d2++) {
    N = (Sample_init_SS + 1.0) + (double)nm1d2;
    zmp_x->data[(int)N - 1] = coef_x->data[4] * cos(coef_x->data[5] * t->data
      [(int)N - 1] + coef_x->data[6]) + coef_x->data[7];
    zmp_y->data[(int)N - 1] = coef_y->data[3] * cos(coef_y->data[4] * t->data
      [(int)N - 1] + coef_y->data[5]);
  }

  /* Fitting for the normal steps */
  N = 2.0;
  jx = 9.0;

  /* for coefx index */
  jy = 7.0;

  /* for coefy index */
  tds = 6.0;

  /* for deltay index */
  while (N < (x + 2.0) - 1.0) {
    Sample_init_SS = (N - 2.0) * Sample_step + Sample_init;

    /* left foot standing */
    ndbl = -w / 2.0 + wfoot / 2.0;
    cdiff = -w / 2.0;
    if (zmp_y_prime0 < 4.0 * (cdiff - ndbl) / tss) {
      Y_Correction(tss, cdiff, ndbl, zmp_y_prime0, dv12);
      for (i12 = 0; i12 < 5; i12++) {
        delta_y->data[(int)(tds + (double)i12) - 1] = dv12[i12];
      }
    } else {
      for (i12 = 0; i12 < 5; i12++) {
        delta_y->data[(int)(tds + (double)i12) - 1] = 0.0;
      }
    }

    Cosine_Cosine_Interpolation(tss, delta12, delta23, zmpLB_x_t->data[(int)
      (Sample_init_SS + 1.0) - 1], zmpUB_x_t->data[(int)(Sample_init_SS + 1.0) -
      1], zmpUB_x_t->data[(int)((Sample_init_SS + Sample_ss) + 1.0) - 1], temp_x);
    for (i12 = 0; i12 < 8; i12++) {
      coef_x->data[(int)(jx + (double)i12) - 1] = temp_x[i12];
    }

    Quadratic_Cosine_Interpolation(tss, delta12, delta23, ndbl, -w / 2.0 + wfoot
      / 2.0, w / 2.0 - wfoot / 2.0, zmp_y_prime0, dv11);
    for (i12 = 0; i12 < 6; i12++) {
      coef_y->data[(int)(jy + (double)i12) - 1] = dv11[i12];
    }

    zmp_y_prime0 = -coef_y->data[(int)(jy + 3.0) - 1] * coef_y->data[(int)(jy +
      4.0) - 1] * sin(coef_y->data[(int)(jy + 4.0) - 1] * tstep + coef_y->data
                      [(int)(jy + 5.0) - 1]);

    /* Single Support of left foot */
    i12 = (int)((Sample_init_SS + Sample_ss) + (1.0 - (Sample_init_SS + 1.0)));
    for (k = 0; k < i12; k++) {
      cdiff = (Sample_init_SS + 1.0) + (double)k;
      zmp_x->data[(int)cdiff - 1] = coef_x->data[(int)jx - 1] * cos(coef_x->
        data[(int)(jx + 1.0) - 1] * (t->data[(int)cdiff - 1] - t->data[(int)
        (Sample_init_SS + 1.0) - 1]) + coef_x->data[(int)(jx + 2.0) - 1]) +
        coef_x->data[(int)(unsigned int)jx + 2];
      ndbl = t->data[(int)cdiff - 1] - t->data[(int)(Sample_init_SS + 1.0) - 1];
      zmp_y->data[(int)cdiff - 1] = (coef_y->data[(int)jy - 1] + coef_y->data
        [(int)(jy + 1.0) - 1] * (t->data[(int)cdiff - 1] - t->data[(int)
        (Sample_init_SS + 1.0) - 1])) + coef_y->data[(int)(jy + 2.0) - 1] *
        (ndbl * ndbl);
      ndbl = t->data[(int)cdiff - 1] - t->data[(int)(Sample_init_SS + 1.0) - 1];
      zmp_y->data[(int)cdiff - 1] = ((((zmp_y->data[(int)cdiff - 1] +
        delta_y->data[(int)tds - 1] * rt_powd_snf(t->data[(int)cdiff - 1] -
        t->data[(int)(Sample_init_SS + 1.0) - 1], 4.0)) + delta_y->data[(int)
        (tds + 1.0) - 1] * rt_powd_snf(t->data[(int)cdiff - 1] - t->data[(int)
        (Sample_init_SS + 1.0) - 1], 3.0)) + delta_y->data[(int)(tds + 2.0) - 1]
        * (ndbl * ndbl)) + delta_y->data[(int)(tds + 3.0) - 1] * (t->data[(int)
        cdiff - 1] - t->data[(int)(Sample_init_SS + 1.0) - 1])) + delta_y->data
        [(int)(unsigned int)tds + 3];
    }

    /* Double Support with right foot in the front */
    ndbl = (Sample_init_SS + Sample_ss) + 1.0;
    i12 = (int)((Sample_init_SS + Sample_step) + (1.0 - ndbl));
    for (k = 0; k < i12; k++) {
      cdiff = ndbl + (double)k;
      zmp_x->data[(int)cdiff - 1] = coef_x->data[(int)(jx + 4.0) - 1] * cos
        (coef_x->data[(int)(jx + 5.0) - 1] * (t->data[(int)cdiff - 1] - t->data
          [(int)(Sample_init_SS + 1.0) - 1]) + coef_x->data[(int)(jx + 6.0) - 1])
        + coef_x->data[(int)(jx + 7.0) - 1];
      zmp_y->data[(int)cdiff - 1] = coef_y->data[(int)(jy + 3.0) - 1] * cos
        (coef_y->data[(int)(jy + 4.0) - 1] * (t->data[(int)cdiff - 1] - t->data
          [(int)(Sample_init_SS + 1.0) - 1]) + coef_y->data[(int)(jy + 5.0) - 1]);
    }

    jx += 8.0;
    jy += 6.0;
    tds += 5.0;
    Sample_init_SS = (N - 1.0) * Sample_step + Sample_init;

    /* right foot standing */
    ndbl = w / 2.0 - wfoot / 2.0;
    cdiff = w / 2.0;
    if (zmp_y_prime0 > 4.0 * (cdiff - ndbl) / tss) {
      Y_Correction(tss, cdiff, ndbl, zmp_y_prime0, dv12);
      for (i12 = 0; i12 < 5; i12++) {
        delta_y->data[(int)(tds + (double)i12) - 1] = dv12[i12];
      }
    } else {
      for (i12 = 0; i12 < 5; i12++) {
        delta_y->data[(int)(tds + (double)i12) - 1] = 0.0;
      }
    }

    Cosine_Cosine_Interpolation(tss, delta12, delta23, zmpLB_x_t->data[(int)
      (Sample_init_SS + 1.0) - 1], zmpUB_x_t->data[(int)(Sample_init_SS + 1.0) -
      1], zmpUB_x_t->data[(int)((Sample_init_SS + Sample_ss) + 1.0) - 1], temp_x);
    for (i12 = 0; i12 < 8; i12++) {
      coef_x->data[(int)(jx + (double)i12) - 1] = temp_x[i12];
    }

    Quadratic_Cosine_Interpolation(tss, delta12, delta23, ndbl, w / 2.0 - wfoot /
      2.0, -w / 2.0 + wfoot / 2.0, zmp_y_prime0, dv11);
    for (i12 = 0; i12 < 6; i12++) {
      coef_y->data[(int)(jy + (double)i12) - 1] = dv11[i12];
    }

    zmp_y_prime0 = -coef_y->data[(int)(jy + 3.0) - 1] * coef_y->data[(int)(jy +
      4.0) - 1] * sin(coef_y->data[(int)(jy + 4.0) - 1] * tstep + coef_y->data
                      [(int)(jy + 5.0) - 1]);

    /* Single Support of right foot */
    i12 = (int)((Sample_init_SS + Sample_ss) + (1.0 - (Sample_init_SS + 1.0)));
    for (k = 0; k < i12; k++) {
      cdiff = (Sample_init_SS + 1.0) + (double)k;
      zmp_x->data[(int)cdiff - 1] = coef_x->data[(int)jx - 1] * cos(coef_x->
        data[(int)(jx + 1.0) - 1] * (t->data[(int)cdiff - 1] - t->data[(int)
        (Sample_init_SS + 1.0) - 1]) + coef_x->data[(int)(jx + 2.0) - 1]) +
        coef_x->data[(int)(unsigned int)jx + 2];
      ndbl = t->data[(int)cdiff - 1] - t->data[(int)(Sample_init_SS + 1.0) - 1];
      zmp_y->data[(int)cdiff - 1] = (coef_y->data[(int)jy - 1] + coef_y->data
        [(int)(jy + 1.0) - 1] * (t->data[(int)cdiff - 1] - t->data[(int)
        (Sample_init_SS + 1.0) - 1])) + coef_y->data[(int)(jy + 2.0) - 1] *
        (ndbl * ndbl);
      ndbl = t->data[(int)cdiff - 1] - t->data[(int)(Sample_init_SS + 1.0) - 1];
      zmp_y->data[(int)cdiff - 1] = ((((zmp_y->data[(int)cdiff - 1] +
        delta_y->data[(int)tds - 1] * rt_powd_snf(t->data[(int)cdiff - 1] -
        t->data[(int)(Sample_init_SS + 1.0) - 1], 4.0)) + delta_y->data[(int)
        (tds + 1.0) - 1] * rt_powd_snf(t->data[(int)cdiff - 1] - t->data[(int)
        (Sample_init_SS + 1.0) - 1], 3.0)) + delta_y->data[(int)(tds + 2.0) - 1]
        * (ndbl * ndbl)) + delta_y->data[(int)(tds + 3.0) - 1] * (t->data[(int)
        cdiff - 1] - t->data[(int)(Sample_init_SS + 1.0) - 1])) + delta_y->data
        [(int)(unsigned int)tds + 3];
    }

    /* Double Support with left foot in the front */
    ndbl = (Sample_init_SS + Sample_ss) + 1.0;
    i12 = (int)((Sample_init_SS + Sample_step) + (1.0 - ndbl));
    for (k = 0; k < i12; k++) {
      cdiff = ndbl + (double)k;
      zmp_x->data[(int)cdiff - 1] = coef_x->data[(int)(jx + 4.0) - 1] * cos
        (coef_x->data[(int)(jx + 5.0) - 1] * (t->data[(int)cdiff - 1] - t->data
          [(int)(Sample_init_SS + 1.0) - 1]) + coef_x->data[(int)(jx + 6.0) - 1])
        + coef_x->data[(int)(jx + 7.0) - 1];
      zmp_y->data[(int)cdiff - 1] = coef_y->data[(int)(jy + 3.0) - 1] * cos
        (coef_y->data[(int)(jy + 4.0) - 1] * (t->data[(int)cdiff - 1] - t->data
          [(int)(Sample_init_SS + 1.0) - 1]) + coef_y->data[(int)(jy + 5.0) - 1]);
    }

    jx += 8.0;
    jy += 6.0;
    tds += 5.0;
    N += 2.0;
  }

  Sample_init_SS = (N - 2.0) * Sample_step + Sample_init;

  /* Sample_End */
  ndbl = zmpLB_x_t->data[(int)(Sample_init_SS + 1.0) - 1] + htar * lfoot;
  Cosine_Cosine_Interpolation(tend, delta12, delta23, zmpLB_x_t->data[(int)
    (Sample_init_SS + 1.0) - 1], ndbl, ndbl, temp_x);
  ZMP_END_FITTING(tend, -w / 2.0 + wfoot / 2.0, -w / 2.0, zmp_y_prime0, dv12);
  for (i12 = 0; i12 < 5; i12++) {
    delta_y->data[(int)(tds + (double)i12) - 1] = dv12[i12];
  }

  for (i12 = 0; i12 < 4; i12++) {
    coef_x->data[(int)(jx + (double)i12) - 1] = temp_x[i12];
  }

  i12 = (int)(Nsample + (1.0 - (Sample_init_SS + 1.0)));
  for (k = 0; k < i12; k++) {
    cdiff = (Sample_init_SS + 1.0) + (double)k;
    zmp_x->data[(int)cdiff - 1] = coef_x->data[(int)jx - 1] * cos(coef_x->data
      [(int)(jx + 1.0) - 1] * (t->data[(int)cdiff - 1] - t->data[(int)
      (Sample_init_SS + 1.0) - 1]) + coef_x->data[(int)(jx + 2.0) - 1]) +
      coef_x->data[(int)(unsigned int)jx + 2];
    ndbl = t->data[(int)cdiff - 1] - t->data[(int)(Sample_init_SS + 1.0) - 1];
    zmp_y->data[(int)cdiff - 1] = (((delta_y->data[(int)tds - 1] * rt_powd_snf
      (t->data[(int)cdiff - 1] - t->data[(int)(Sample_init_SS + 1.0) - 1], 4.0)
      + delta_y->data[(int)(tds + 1.0) - 1] * rt_powd_snf(t->data[(int)cdiff - 1]
      - t->data[(int)(Sample_init_SS + 1.0) - 1], 3.0)) + delta_y->data[(int)
      (tds + 2.0) - 1] * (ndbl * ndbl)) + delta_y->data[(int)(tds + 3.0) - 1] *
                                   (t->data[(int)cdiff - 1] - t->data[(int)
      (Sample_init_SS + 1.0) - 1])) + delta_y->data[(int)(unsigned int)tds + 3];
  }

  emxFree_real_T(&t);

  /* { */
  /* figure(4); */
  /* subplot(2, 1, 1); */
  /* plot(t, zmp_x); */
  /* ylabel('x(m)'); */
  /* xlabel('t(s)'); */
  /* title('ZMP Generation--X'); */
  /*  */
  /* subplot(2, 1, 2); */
  /* plot(t, zmp_y); */
  /* ylabel('y(m)'); */
  /* xlabel('t(s)'); */
  /* set(gca,'ydir','reverse'); */
  /* title('ZMP Generation--Y'); */
  /*  */
  /* figure(2); */
  /* hold on; */
  /* for i=1:Nsample */
  /*     scatter(zmp_x(i),zmp_y(i),'kx'); */
  /* end */
  /* hold off; */
  /* } */
}

/* End of code generation (ZMP_Generation.c) */
