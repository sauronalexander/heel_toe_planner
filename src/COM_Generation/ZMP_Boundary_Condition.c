/*
 * ZMP_Boundary_Condition.c
 *
 * Code generation for function 'ZMP_Boundary_Condition'
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
#include <stdio.h>

/* Function Definitions */
void test_zmp_boundary()
{
    printf("Testing ZMP Boundary Condition\n");
}

void ZMP_Boundary_Condition(double totaltime, double tinit, double tend, double
                            tstep, double ratio, double Period, double unusedU0, double lfoot, double w,
                            double wfoot, emxArray_real_T * leftgait_x, emxArray_real_T * rightgait_x,
                            emxArray_real_T * leftgait_y, emxArray_real_T * rightgait_y,
                            emxArray_real_T * zmpUB_x, emxArray_real_T * zmpLB_x, emxArray_real_T * zmpUB_y,
                            emxArray_real_T * zmpLB_y, emxArray_real_T * zmpUB_x_t, emxArray_real_T * zmpLB_x_t)
{


  double x;
  double Sample_step;
  double Sample_init_SS;
  double Nsample;
  double Sample_init;
  double Sample_ss;
  double kd;
  int n;
  double x_lower;
  double offset;
  double ndbl;
  double cdiff;
  emxArray_real_T *y;
  int i10;
  int nm1d2;
  int idx;
  unsigned int xlength[2];
  emxArray_boolean_T *b_x;
  emxArray_int32_T *ii;
  boolean_T exitg7;
  boolean_T guard7 = FALSE;
  emxArray_int32_T *b_ii;
  emxArray_real_T *b_index;
  double i;
  boolean_T exitg6;
  boolean_T guard6 = FALSE;
  emxArray_int32_T *c_ii;
  emxArray_int32_T *d_ii;
  emxArray_int32_T *e_ii;
  emxArray_int32_T *f_ii;
  emxArray_int32_T *g_ii;
  boolean_T exitg5;
  boolean_T guard5 = FALSE;
  boolean_T exitg4;
  boolean_T guard4 = FALSE;
  boolean_T exitg3;
  boolean_T guard3 = FALSE;
  boolean_T exitg2;
  boolean_T guard2 = FALSE;
  boolean_T exitg1;
  boolean_T guard1 = FALSE;
  emxArray_int32_T *h_ii;
  (void)unusedU0;

  /* %ZMP Boundary Condition */
  /* input: totaltime, tinit, tend, tstep, ratio, Period(for sample), lstep, lfoot, w(hipwidth), wfoot, leftgait_x, rightgait_x, leftgait_y, rightgait_y; */
  /* output: zmpUB_x, zmpLB_x, zmpUB_y, zmpLB_y  */
  /* General Initialization */
  x = floor(((totaltime - tinit) - tend) / tstep);

  /* Number of steps */
  Sample_step = floor(tstep / Period);
  Sample_init_SS = floor(10.0 * tinit / (10.0 * Period));
  Nsample = (((x + 2.0) - 2.0) * Sample_step + Sample_init_SS) + (floor(tend /
    Period) + 1.0);

  /* Number of samples  */
  Sample_init = Sample_init_SS + floor(ratio * tstep / Period);
  Sample_ss = floor((1.0 - ratio) * tstep / Period);

  /* initiallization */
  kd = rightgait_x->data[(int)Nsample - 1] + lfoot;
  if (rtIsNaN(kd)) {
    n = 0;
    x_lower = rtNaN;
    offset = kd;
  } else if (kd < 0.0) {
    n = -1;
    x_lower = 0.0;
    offset = kd;
  } else if (rtIsInf(kd)) {
    n = 0;
    x_lower = rtNaN;
    offset = kd;
  } else {
    x_lower = 0.0;
    ndbl = floor(kd / 0.01 + 0.5);
    offset = ndbl * 0.01;
    cdiff = offset - kd;
    if (fabs(cdiff) < 4.4408920985006262E-16 * kd) {
      ndbl++;
      offset = kd;
    } else if (cdiff > 0.0) {
      offset = (ndbl - 1.0) * 0.01;
    } else {
      ndbl++;
    }

    if (ndbl >= 0.0) {
      n = (int)ndbl - 1;
    } else {
      n = -1;
    }
  }

  int i6;
  int loop_ub;
  i6 = zmpUB_x->size[0] * zmpUB_x->size[1];
  zmpUB_x->size[0] = 1;
  zmpUB_x->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)zmpUB_x, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    zmpUB_x->data[i6] = 0.0;
  }

  i6 = zmpLB_x->size[0] * zmpLB_x->size[1];
  zmpLB_x->size[0] = 1;
  zmpLB_x->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)zmpLB_x, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    zmpLB_x->data[i6] = 0.0;
  }

  i6 = zmpUB_y->size[0] * zmpUB_y->size[1];
  zmpUB_y->size[0] = 1;
  zmpUB_y->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)zmpUB_y, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    zmpUB_y->data[i6] = 0.0;
  }

  i6 = zmpLB_y->size[0] * zmpLB_y->size[1];
  zmpLB_y->size[0] = 1;
  zmpLB_y->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)zmpLB_y, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    zmpLB_y->data[i6] = 0.0;
  }

  i6 = zmpLB_x_t->size[0] * zmpLB_x_t->size[1];
  zmpLB_x_t->size[0] = 1;
  zmpLB_x_t->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)zmpLB_x_t, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    zmpLB_x_t->data[i6] = 0.0;
  }

  i6 = zmpUB_x_t->size[0] * zmpUB_x_t->size[1];
  zmpUB_x_t->size[0] = 1;
  zmpUB_x_t->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)zmpUB_x_t, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    zmpUB_x_t->data[i6] = 0.0;
  }


  emxInit_real_T(&y, 2);
  i10 = y->size[0] * y->size[1];
  y->size[0] = 1;
  y->size[1] = n + 1;
  emxEnsureCapacity((emxArray__common *)y, i10, (int)sizeof(double));
  if (n + 1 > 0) {
    y->data[0] = x_lower;
    if (n + 1 > 1) {
      y->data[n] = offset;
      nm1d2 = n / 2;
      for (idx = 1; idx < nm1d2; idx++) {
        kd = (double)idx * 0.01;
        y->data[idx] = x_lower + kd;
        y->data[n - idx] = offset - kd;
      }

      if (nm1d2 << 1 == n) {
        y->data[nm1d2] = (x_lower + offset) / 2.0;
      } else {
        kd = (double)nm1d2 * 0.01;
        y->data[nm1d2] = x_lower + kd;
        y->data[nm1d2 + 1] = offset - kd;
      }
    }
  }

  nm1d2 = (int)Nsample;
  for (i10 = 0; i10 < nm1d2; i10++) {
    zmpUB_x_t->data[i10] = 0.0;
  }

  nm1d2 = (int)Nsample;
  for (i10 = 0; i10 < nm1d2; i10++) {
    zmpLB_x_t->data[i10] = 0.0;
  }

  for (i10 = 0; i10 < 2; i10++) {
    xlength[i10] = (unsigned int)y->size[i10];
  }

  i10 = zmpUB_x->size[0] * zmpUB_x->size[1];
  zmpUB_x->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)zmpUB_x, i10, (int)sizeof(double));
  i10 = zmpUB_x->size[0] * zmpUB_x->size[1];
  zmpUB_x->size[1] = (int)xlength[1];
  emxEnsureCapacity((emxArray__common *)zmpUB_x, i10, (int)sizeof(double));
  nm1d2 = (int)xlength[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    zmpUB_x->data[i10] = 0.0;
  }

  i10 = zmpUB_y->size[0] * zmpUB_y->size[1];
  zmpUB_y->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)zmpUB_y, i10, (int)sizeof(double));
  i10 = zmpUB_y->size[0] * zmpUB_y->size[1];
  zmpUB_y->size[1] = (int)xlength[1];
  emxEnsureCapacity((emxArray__common *)zmpUB_y, i10, (int)sizeof(double));
  nm1d2 = (int)xlength[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    zmpUB_y->data[i10] = 0.0;
  }

  i10 = zmpLB_x->size[0] * zmpLB_x->size[1];
  zmpLB_x->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)zmpLB_x, i10, (int)sizeof(double));
  i10 = zmpLB_x->size[0] * zmpLB_x->size[1];
  zmpLB_x->size[1] = (int)xlength[1];
  emxEnsureCapacity((emxArray__common *)zmpLB_x, i10, (int)sizeof(double));
  nm1d2 = (int)xlength[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    zmpLB_x->data[i10] = 0.0;
  }

  i10 = zmpLB_y->size[0] * zmpLB_y->size[1];
  zmpLB_y->size[0] = 1;
  emxEnsureCapacity((emxArray__common *)zmpLB_y, i10, (int)sizeof(double));
  i10 = zmpLB_y->size[0] * zmpLB_y->size[1];
  zmpLB_y->size[1] = (int)xlength[1];
  emxEnsureCapacity((emxArray__common *)zmpLB_y, i10, (int)sizeof(double));
  nm1d2 = (int)xlength[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    zmpLB_y->data[i10] = 0.0;
  }

  /* figure(2); */
  /* title('ZMP Boundary in x-y plane'); */
  /* hold on; */
  /* Initial State */
  /* Right foot standing, left foot moving */
  for (nm1d2 = 0; nm1d2 < (int)Sample_init_SS; nm1d2++) {
    zmpUB_x_t->data[nm1d2] = rightgait_x->data[nm1d2] + lfoot;
    zmpLB_x_t->data[nm1d2] = rightgait_x->data[nm1d2];
  }

  emxInit_boolean_T(&b_x, 2);
  i10 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i10, (int)sizeof(boolean_T));
  kd = zmpUB_x_t->data[(int)Sample_init_SS - 1];
  ndbl = zmpLB_x_t->data[(int)Sample_init_SS - 1];
  nm1d2 = y->size[0] * y->size[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    b_x->data[i10] = ((y->data[i10] <= kd) && (y->data[i10] >= ndbl));
  }

  emxInit_int32_T(&ii, 2);
  idx = 0;
  i10 = ii->size[0] * ii->size[1];
  ii->size[0] = 1;
  ii->size[1] = b_x->size[1];
  emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
  nm1d2 = 1;
  exitg7 = FALSE;
  while ((exitg7 == FALSE) && (nm1d2 <= b_x->size[1])) {
    guard7 = FALSE;
    if (b_x->data[nm1d2 - 1]) {
      idx++;
      ii->data[idx - 1] = nm1d2;
      if (idx >= b_x->size[1]) {
        exitg7 = TRUE;
      } else {
        guard7 = TRUE;
      }
    } else {
      guard7 = TRUE;
    }

    if (guard7 == TRUE) {
      nm1d2++;
    }
  }

  if (b_x->size[1] == 1) {
    if (idx == 0) {
      i10 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    }
  } else {
    if (1 > idx) {
      nm1d2 = 0;
    } else {
      nm1d2 = idx;
    }

    emxInit_int32_T(&b_ii, 2);
    i10 = b_ii->size[0] * b_ii->size[1];
    b_ii->size[0] = 1;
    b_ii->size[1] = nm1d2;
    emxEnsureCapacity((emxArray__common *)b_ii, i10, (int)sizeof(int));
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_ii->data[b_ii->size[0] * i10] = ii->data[i10];
    }

    i10 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = b_ii->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    nm1d2 = b_ii->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      ii->data[ii->size[0] * i10] = b_ii->data[b_ii->size[0] * i10];
    }

    emxFree_int32_T(&b_ii);
  }

  emxInit_real_T(&b_index, 2);
  i10 = b_index->size[0] * b_index->size[1];
  b_index->size[0] = 1;
  b_index->size[1] = ii->size[1];
  emxEnsureCapacity((emxArray__common *)b_index, i10, (int)sizeof(double));
  nm1d2 = ii->size[0] * ii->size[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    b_index->data[i10] = ii->data[i10];
  }

  i10 = b_index->size[1];
  for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
    zmpUB_x->data[(int)b_index->data[nm1d2] - 1] = rightgait_x->data[(int)
      Sample_init_SS - 1] + lfoot;
    zmpLB_x->data[(int)b_index->data[nm1d2] - 1] = rightgait_x->data[(int)
      Sample_init_SS - 1];
    zmpUB_y->data[(int)b_index->data[nm1d2] - 1] = rightgait_y->data[(int)
      Sample_init_SS - 1] + wfoot / 2.0;
    zmpLB_y->data[(int)b_index->data[nm1d2] - 1] = rightgait_y->data[(int)
      Sample_init_SS - 1] - wfoot / 2.0;
  }

  /* line([zmpLB_x_t(Sample_init_SS), zmpLB_x_t(Sample_init_SS)], [rightgait_y(Sample_init_SS) - wfoot/2, rightgait_y(Sample_init_SS) + wfoot/2]); */
  /* line([zmpUB_x_t(Sample_init_SS), zmpUB_x_t(Sample_init_SS)], [rightgait_y(Sample_init_SS) - wfoot/2, rightgait_y(Sample_init_SS) + wfoot/2]); */
  /* Double Support in Init State */
  x_lower = rightgait_x->data[(int)Sample_init_SS - 1] + lfoot;
  i10 = (int)(Sample_init + (1.0 - (Sample_init_SS + 1.0)));
  for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
    i = (Sample_init_SS + 1.0) + (double)nm1d2;
    zmpUB_x_t->data[(int)i - 1] = leftgait_x->data[(int)(Sample_init + 1.0) - 1];
    zmpLB_x_t->data[(int)i - 1] = x_lower;
  }

  kd = -1.0 / (leftgait_x->data[(int)(Sample_init + 1.0) - 1] - x_lower);
  i10 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i10, (int)sizeof(boolean_T));
  nm1d2 = y->size[0] * y->size[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    b_x->data[i10] = ((y->data[i10] <= leftgait_x->data[(int)(Sample_init + 1.0)
                       - 1]) && (y->data[i10] >= x_lower));
  }

  idx = 0;
  i10 = ii->size[0] * ii->size[1];
  ii->size[0] = 1;
  ii->size[1] = b_x->size[1];
  emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
  nm1d2 = 1;
  exitg6 = FALSE;
  while ((exitg6 == FALSE) && (nm1d2 <= b_x->size[1])) {
    guard6 = FALSE;
    if (b_x->data[nm1d2 - 1]) {
      idx++;
      ii->data[idx - 1] = nm1d2;
      if (idx >= b_x->size[1]) {
        exitg6 = TRUE;
      } else {
        guard6 = TRUE;
      }
    } else {
      guard6 = TRUE;
    }

    if (guard6 == TRUE) {
      nm1d2++;
    }
  }

  if (b_x->size[1] == 1) {
    if (idx == 0) {
      i10 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    }
  } else {
    if (1 > idx) {
      nm1d2 = 0;
    } else {
      nm1d2 = idx;
    }

    emxInit_int32_T(&c_ii, 2);
    i10 = c_ii->size[0] * c_ii->size[1];
    c_ii->size[0] = 1;
    c_ii->size[1] = nm1d2;
    emxEnsureCapacity((emxArray__common *)c_ii, i10, (int)sizeof(int));
    for (i10 = 0; i10 < nm1d2; i10++) {
      c_ii->data[c_ii->size[0] * i10] = ii->data[i10];
    }

    i10 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = c_ii->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    nm1d2 = c_ii->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      ii->data[ii->size[0] * i10] = c_ii->data[c_ii->size[0] * i10];
    }

    emxFree_int32_T(&c_ii);
  }

  i10 = b_index->size[0] * b_index->size[1];
  b_index->size[0] = 1;
  b_index->size[1] = ii->size[1];
  emxEnsureCapacity((emxArray__common *)b_index, i10, (int)sizeof(double));
  nm1d2 = ii->size[0] * ii->size[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    b_index->data[i10] = ii->data[i10];
  }

  i10 = b_index->size[1];
  for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
    zmpUB_x->data[(int)b_index->data[nm1d2] - 1] = leftgait_x->data[(int)
      (Sample_init + 1.0) - 1];
    zmpLB_x->data[(int)b_index->data[nm1d2] - 1] = x_lower;
    zmpUB_y->data[(int)b_index->data[nm1d2] - 1] = kd * ((w * y->data[(int)
      b_index->data[nm1d2] - 1] - w / 2.0 * (leftgait_x->data[(int)(Sample_init +
      1.0) - 1] + x_lower)) - wfoot / 2.0 * (leftgait_x->data[(int)(Sample_init +
      1.0) - 1] - x_lower));
    zmpLB_y->data[(int)b_index->data[nm1d2] - 1] = kd * ((w * y->data[(int)
      b_index->data[nm1d2] - 1] - w / 2.0 * (leftgait_x->data[(int)(Sample_init +
      1.0) - 1] + x_lower)) + wfoot / 2.0 * (leftgait_x->data[(int)(Sample_init +
      1.0) - 1] - x_lower));
  }

  /* line([x_upper, x_upper], [-w/2+wfoot/2, -w/2-wfoot/2]); */
  /* Cyclic Gaits */
  i = 2.0;
  offset = 0.0 * Sample_step + Sample_init;
  emxInit_int32_T(&d_ii, 2);
  emxInit_int32_T(&e_ii, 2);
  emxInit_int32_T(&f_ii, 2);
  emxInit_int32_T(&g_ii, 2);
  while (i < (x + 2.0) - 1.0) {
    /* Single Support of left foot */
    offset = (i - 2.0) * Sample_step + Sample_init;
    i10 = (int)((offset + Sample_ss) + (1.0 - (offset + 1.0)));
    for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
      cdiff = (offset + 1.0) + (double)nm1d2;
      zmpUB_x_t->data[(int)cdiff - 1] = leftgait_x->data[(int)(offset + 1.0) - 1]
        + lfoot;
      zmpLB_x_t->data[(int)cdiff - 1] = leftgait_x->data[(int)(offset + 1.0) - 1];
    }

    kd = leftgait_x->data[(int)(offset + 1.0) - 1] + lfoot;
    i10 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = 1;
    b_x->size[1] = y->size[1];
    emxEnsureCapacity((emxArray__common *)b_x, i10, (int)sizeof(boolean_T));
    nm1d2 = y->size[0] * y->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_x->data[i10] = ((y->data[i10] <= kd) && (y->data[i10] >=
        leftgait_x->data[(int)(offset + 1.0) - 1]));
    }

    idx = 0;
    i10 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = b_x->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    nm1d2 = 1;
    exitg5 = FALSE;
    while ((exitg5 == FALSE) && (nm1d2 <= b_x->size[1])) {
      guard5 = FALSE;
      if (b_x->data[nm1d2 - 1]) {
        idx++;
        ii->data[idx - 1] = nm1d2;
        if (idx >= b_x->size[1]) {
          exitg5 = TRUE;
        } else {
          guard5 = TRUE;
        }
      } else {
        guard5 = TRUE;
      }

      if (guard5 == TRUE) {
        nm1d2++;
      }
    }

    if (b_x->size[1] == 1) {
      if (idx == 0) {
        i10 = ii->size[0] * ii->size[1];
        ii->size[0] = 1;
        ii->size[1] = 0;
        emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
      }
    } else {
      if (1 > idx) {
        nm1d2 = 0;
      } else {
        nm1d2 = idx;
      }

      i10 = d_ii->size[0] * d_ii->size[1];
      d_ii->size[0] = 1;
      d_ii->size[1] = nm1d2;
      emxEnsureCapacity((emxArray__common *)d_ii, i10, (int)sizeof(int));
      for (i10 = 0; i10 < nm1d2; i10++) {
        d_ii->data[d_ii->size[0] * i10] = ii->data[i10];
      }

      i10 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = d_ii->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
      nm1d2 = d_ii->size[1];
      for (i10 = 0; i10 < nm1d2; i10++) {
        ii->data[ii->size[0] * i10] = d_ii->data[d_ii->size[0] * i10];
      }
    }

    i10 = b_index->size[0] * b_index->size[1];
    b_index->size[0] = 1;
    b_index->size[1] = ii->size[1];
    emxEnsureCapacity((emxArray__common *)b_index, i10, (int)sizeof(double));
    nm1d2 = ii->size[0] * ii->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_index->data[i10] = ii->data[i10];
    }

    i10 = b_index->size[1];
    for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
      zmpUB_x->data[(int)b_index->data[nm1d2] - 1] = leftgait_x->data[(int)
        (offset + 1.0) - 1] + lfoot;
      zmpLB_x->data[(int)b_index->data[nm1d2] - 1] = leftgait_x->data[(int)
        (offset + 1.0) - 1];
      zmpUB_y->data[(int)b_index->data[nm1d2] - 1] = leftgait_y->data[(int)
        (offset + 1.0) - 1] + wfoot / 2.0;
      zmpLB_y->data[(int)b_index->data[nm1d2] - 1] = leftgait_y->data[(int)
        (offset + 1.0) - 1] - wfoot / 2.0;
    }

    /* line([leftgait_x(offset+1) + lfoot, leftgait_x(offset+1) + lfoot], [leftgait_y(offset+1) + wfoot/2, leftgait_y(offset+1) - wfoot/2]); */
    /* First Double Support */
    x_lower = leftgait_x->data[(int)((offset + Sample_ss) + 1.0) - 1] + lfoot;
    kd = 1.0 / (rightgait_x->data[(int)((offset + Sample_ss) + 1.0) - 1] -
                x_lower);
    ndbl = (offset + 1.0) + Sample_ss;
    i10 = (int)((offset + Sample_step) + (1.0 - ndbl));
    for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
      cdiff = ndbl + (double)nm1d2;
      zmpUB_x_t->data[(int)cdiff - 1] = rightgait_x->data[(int)((offset +
        Sample_ss) + 1.0) - 1];
      zmpLB_x_t->data[(int)cdiff - 1] = x_lower;
    }

    i10 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = 1;
    b_x->size[1] = y->size[1];
    emxEnsureCapacity((emxArray__common *)b_x, i10, (int)sizeof(boolean_T));
    nm1d2 = y->size[0] * y->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_x->data[i10] = ((y->data[i10] <= rightgait_x->data[(int)((offset +
        Sample_ss) + 1.0) - 1]) && (y->data[i10] >= x_lower));
    }

    idx = 0;
    i10 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = b_x->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    nm1d2 = 1;
    exitg4 = FALSE;
    while ((exitg4 == FALSE) && (nm1d2 <= b_x->size[1])) {
      guard4 = FALSE;
      if (b_x->data[nm1d2 - 1]) {
        idx++;
        ii->data[idx - 1] = nm1d2;
        if (idx >= b_x->size[1]) {
          exitg4 = TRUE;
        } else {
          guard4 = TRUE;
        }
      } else {
        guard4 = TRUE;
      }

      if (guard4 == TRUE) {
        nm1d2++;
      }
    }

    if (b_x->size[1] == 1) {
      if (idx == 0) {
        i10 = ii->size[0] * ii->size[1];
        ii->size[0] = 1;
        ii->size[1] = 0;
        emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
      }
    } else {
      if (1 > idx) {
        nm1d2 = 0;
      } else {
        nm1d2 = idx;
      }

      i10 = e_ii->size[0] * e_ii->size[1];
      e_ii->size[0] = 1;
      e_ii->size[1] = nm1d2;
      emxEnsureCapacity((emxArray__common *)e_ii, i10, (int)sizeof(int));
      for (i10 = 0; i10 < nm1d2; i10++) {
        e_ii->data[e_ii->size[0] * i10] = ii->data[i10];
      }

      i10 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = e_ii->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
      nm1d2 = e_ii->size[1];
      for (i10 = 0; i10 < nm1d2; i10++) {
        ii->data[ii->size[0] * i10] = e_ii->data[e_ii->size[0] * i10];
      }
    }

    i10 = b_index->size[0] * b_index->size[1];
    b_index->size[0] = 1;
    b_index->size[1] = ii->size[1];
    emxEnsureCapacity((emxArray__common *)b_index, i10, (int)sizeof(double));
    nm1d2 = ii->size[0] * ii->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_index->data[i10] = ii->data[i10];
    }

    i10 = b_index->size[1];
    for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
      zmpUB_x->data[(int)b_index->data[nm1d2] - 1] = rightgait_x->data[(int)
        ((offset + Sample_ss) + 1.0) - 1];
      zmpLB_x->data[(int)b_index->data[nm1d2] - 1] = x_lower;
      zmpUB_y->data[(int)b_index->data[nm1d2] - 1] = kd * ((w * y->data[(int)
        b_index->data[nm1d2] - 1] - w / 2.0 * (rightgait_x->data[(int)((offset +
        Sample_ss) + 1.0) - 1] + x_lower)) + wfoot / 2.0 * (rightgait_x->data
        [(int)((offset + Sample_ss) + 1.0) - 1] - x_lower));
      zmpLB_y->data[(int)b_index->data[nm1d2] - 1] = kd * ((w * y->data[(int)
        b_index->data[nm1d2] - 1] - w / 2.0 * (rightgait_x->data[(int)((offset +
        Sample_ss) + 1.0) - 1] + x_lower)) - wfoot / 2.0 * (rightgait_x->data
        [(int)((offset + Sample_ss) + 1.0) - 1] - x_lower));
    }

    /* line([x_upper, x_upper],[w/2+wfoot/2, w/2-wfoot/2]); */
    /* Single Support of right foot */
    offset = (i - 1.0) * Sample_step + Sample_init;
    i10 = (int)((offset + Sample_ss) + (1.0 - (offset + 1.0)));
    for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
      cdiff = (offset + 1.0) + (double)nm1d2;
      zmpUB_x_t->data[(int)cdiff - 1] = rightgait_x->data[(int)(offset + 1.0) - 1]
        + lfoot;
      zmpLB_x_t->data[(int)cdiff - 1] = rightgait_x->data[(int)(offset + 1.0) - 1];
    }

    i10 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = 1;
    b_x->size[1] = y->size[1];
    emxEnsureCapacity((emxArray__common *)b_x, i10, (int)sizeof(boolean_T));
    kd = zmpUB_x_t->data[(int)(offset + 1.0) - 1];
    ndbl = zmpLB_x_t->data[(int)(offset + 1.0) - 1];
    nm1d2 = y->size[0] * y->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_x->data[i10] = ((y->data[i10] <= kd) && (y->data[i10] >= ndbl));
    }

    idx = 0;
    i10 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = b_x->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    nm1d2 = 1;
    exitg3 = FALSE;
    while ((exitg3 == FALSE) && (nm1d2 <= b_x->size[1])) {
      guard3 = FALSE;
      if (b_x->data[nm1d2 - 1]) {
        idx++;
        ii->data[idx - 1] = nm1d2;
        if (idx >= b_x->size[1]) {
          exitg3 = TRUE;
        } else {
          guard3 = TRUE;
        }
      } else {
        guard3 = TRUE;
      }

      if (guard3 == TRUE) {
        nm1d2++;
      }
    }

    if (b_x->size[1] == 1) {
      if (idx == 0) {
        i10 = ii->size[0] * ii->size[1];
        ii->size[0] = 1;
        ii->size[1] = 0;
        emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
      }
    } else {
      if (1 > idx) {
        nm1d2 = 0;
      } else {
        nm1d2 = idx;
      }

      i10 = f_ii->size[0] * f_ii->size[1];
      f_ii->size[0] = 1;
      f_ii->size[1] = nm1d2;
      emxEnsureCapacity((emxArray__common *)f_ii, i10, (int)sizeof(int));
      for (i10 = 0; i10 < nm1d2; i10++) {
        f_ii->data[f_ii->size[0] * i10] = ii->data[i10];
      }

      i10 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = f_ii->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
      nm1d2 = f_ii->size[1];
      for (i10 = 0; i10 < nm1d2; i10++) {
        ii->data[ii->size[0] * i10] = f_ii->data[f_ii->size[0] * i10];
      }
    }

    i10 = b_index->size[0] * b_index->size[1];
    b_index->size[0] = 1;
    b_index->size[1] = ii->size[1];
    emxEnsureCapacity((emxArray__common *)b_index, i10, (int)sizeof(double));
    nm1d2 = ii->size[0] * ii->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_index->data[i10] = ii->data[i10];
    }

    i10 = b_index->size[1];
    for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
      zmpUB_x->data[(int)b_index->data[nm1d2] - 1] = rightgait_x->data[(int)
        (offset + 1.0) - 1] + lfoot;
      zmpLB_x->data[(int)b_index->data[nm1d2] - 1] = rightgait_x->data[(int)
        (offset + 1.0) - 1];
      zmpUB_y->data[(int)b_index->data[nm1d2] - 1] = rightgait_y->data[(int)
        (offset + 1.0) - 1] + wfoot / 2.0;
      zmpLB_y->data[(int)b_index->data[nm1d2] - 1] = rightgait_y->data[(int)
        (offset + 1.0) - 1] - wfoot / 2.0;
    }

    /* line([rightgait_x(offset+1) + lfoot, rightgait_x(offset+1) + lfoot], [rightgait_y(offset+1) + wfoot/2, rightgait_y(offset+1) - wfoot/2]); */
    /* Second Double Support */
    x_lower = rightgait_x->data[(int)((offset + Sample_ss) + 1.0) - 1] + lfoot;
    kd = -1.0 / (leftgait_x->data[(int)((offset + Sample_ss) + 1.0) - 1] -
                 x_lower);
    ndbl = (offset + 1.0) + Sample_ss;
    i10 = (int)((offset + Sample_step) + (1.0 - ndbl));
    for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
      cdiff = ndbl + (double)nm1d2;
      zmpUB_x_t->data[(int)cdiff - 1] = leftgait_x->data[(int)((offset + Sample_ss)
        + 1.0) - 1];
      zmpLB_x_t->data[(int)cdiff - 1] = x_lower;
    }

    i10 = b_x->size[0] * b_x->size[1];
    b_x->size[0] = 1;
    b_x->size[1] = y->size[1];
    emxEnsureCapacity((emxArray__common *)b_x, i10, (int)sizeof(boolean_T));
    nm1d2 = y->size[0] * y->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_x->data[i10] = ((y->data[i10] <= leftgait_x->data[(int)((offset +
        Sample_ss) + 1.0) - 1]) && (y->data[i10] >= x_lower));
    }

    idx = 0;
    i10 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = b_x->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    nm1d2 = 1;
    exitg2 = FALSE;
    while ((exitg2 == FALSE) && (nm1d2 <= b_x->size[1])) {
      guard2 = FALSE;
      if (b_x->data[nm1d2 - 1]) {
        idx++;
        ii->data[idx - 1] = nm1d2;
        if (idx >= b_x->size[1]) {
          exitg2 = TRUE;
        } else {
          guard2 = TRUE;
        }
      } else {
        guard2 = TRUE;
      }

      if (guard2 == TRUE) {
        nm1d2++;
      }
    }

    if (b_x->size[1] == 1) {
      if (idx == 0) {
        i10 = ii->size[0] * ii->size[1];
        ii->size[0] = 1;
        ii->size[1] = 0;
        emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
      }
    } else {
      if (1 > idx) {
        nm1d2 = 0;
      } else {
        nm1d2 = idx;
      }

      i10 = g_ii->size[0] * g_ii->size[1];
      g_ii->size[0] = 1;
      g_ii->size[1] = nm1d2;
      emxEnsureCapacity((emxArray__common *)g_ii, i10, (int)sizeof(int));
      for (i10 = 0; i10 < nm1d2; i10++) {
        g_ii->data[g_ii->size[0] * i10] = ii->data[i10];
      }

      i10 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = g_ii->size[1];
      emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
      nm1d2 = g_ii->size[1];
      for (i10 = 0; i10 < nm1d2; i10++) {
        ii->data[ii->size[0] * i10] = g_ii->data[g_ii->size[0] * i10];
      }
    }

    i10 = b_index->size[0] * b_index->size[1];
    b_index->size[0] = 1;
    b_index->size[1] = ii->size[1];
    emxEnsureCapacity((emxArray__common *)b_index, i10, (int)sizeof(double));
    nm1d2 = ii->size[0] * ii->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      b_index->data[i10] = ii->data[i10];
    }

    i10 = b_index->size[1];
    for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
      zmpUB_x->data[(int)b_index->data[nm1d2] - 1] = leftgait_x->data[(int)
        ((offset + Sample_ss) + 1.0) - 1];
      zmpLB_x->data[(int)b_index->data[nm1d2] - 1] = x_lower;
      zmpUB_y->data[(int)b_index->data[nm1d2] - 1] = kd * ((w * y->data[(int)
        b_index->data[nm1d2] - 1] - w / 2.0 * (leftgait_x->data[(int)((offset +
        Sample_ss) + 1.0) - 1] + x_lower)) - wfoot / 2.0 * (leftgait_x->data[(int)
        ((offset + Sample_ss) + 1.0) - 1] - x_lower));
      zmpLB_y->data[(int)b_index->data[nm1d2] - 1] = kd * ((w * y->data[(int)
        b_index->data[nm1d2] - 1] - w / 2.0 * (leftgait_x->data[(int)((offset +
        Sample_ss) + 1.0) - 1] + x_lower)) + wfoot / 2.0 * (leftgait_x->data[(int)
        ((offset + Sample_ss) + 1.0) - 1] - x_lower));
    }

    /* line([x_upper, x_upper],[-w/2+wfoot/2, -w/2-wfoot/2]); */
    i += 2.0;
  }

  emxFree_int32_T(&g_ii);
  emxFree_int32_T(&f_ii);
  emxFree_int32_T(&e_ii);
  emxFree_int32_T(&d_ii);

  /* End State */
  /* left foot standing, right foot moving */
  offset += Sample_step;
  i10 = (int)(Nsample + (1.0 - (offset + 1.0)));
  for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
    i = (offset + 1.0) + (double)nm1d2;
    zmpUB_x_t->data[(int)i - 1] = leftgait_x->data[(int)(offset + 1.0) - 1] +
      lfoot;
    zmpLB_x_t->data[(int)i - 1] = leftgait_x->data[(int)(offset + 1.0) - 1];
  }

  i10 = b_x->size[0] * b_x->size[1];
  b_x->size[0] = 1;
  b_x->size[1] = y->size[1];
  emxEnsureCapacity((emxArray__common *)b_x, i10, (int)sizeof(boolean_T));
  kd = zmpUB_x_t->data[(int)(offset + 1.0) - 1];
  ndbl = zmpLB_x_t->data[(int)(offset + 1.0) - 1];
  nm1d2 = y->size[0] * y->size[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    b_x->data[i10] = ((y->data[i10] <= kd) && (y->data[i10] >= ndbl));
  }

  emxFree_real_T(&y);
  idx = 0;
  i10 = ii->size[0] * ii->size[1];
  ii->size[0] = 1;
  ii->size[1] = b_x->size[1];
  emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
  nm1d2 = 1;
  exitg1 = FALSE;
  while ((exitg1 == FALSE) && (nm1d2 <= b_x->size[1])) {
    guard1 = FALSE;
    if (b_x->data[nm1d2 - 1]) {
      idx++;
      ii->data[idx - 1] = nm1d2;
      if (idx >= b_x->size[1]) {
        exitg1 = TRUE;
      } else {
        guard1 = TRUE;
      }
    } else {
      guard1 = TRUE;
    }

    if (guard1 == TRUE) {
      nm1d2++;
    }
  }

  if (b_x->size[1] == 1) {
    if (idx == 0) {
      i10 = ii->size[0] * ii->size[1];
      ii->size[0] = 1;
      ii->size[1] = 0;
      emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    }
  } else {
    if (1 > idx) {
      nm1d2 = 0;
    } else {
      nm1d2 = idx;
    }

    emxInit_int32_T(&h_ii, 2);
    i10 = h_ii->size[0] * h_ii->size[1];
    h_ii->size[0] = 1;
    h_ii->size[1] = nm1d2;
    emxEnsureCapacity((emxArray__common *)h_ii, i10, (int)sizeof(int));
    for (i10 = 0; i10 < nm1d2; i10++) {
      h_ii->data[h_ii->size[0] * i10] = ii->data[i10];
    }

    i10 = ii->size[0] * ii->size[1];
    ii->size[0] = 1;
    ii->size[1] = h_ii->size[1];
    emxEnsureCapacity((emxArray__common *)ii, i10, (int)sizeof(int));
    nm1d2 = h_ii->size[1];
    for (i10 = 0; i10 < nm1d2; i10++) {
      ii->data[ii->size[0] * i10] = h_ii->data[h_ii->size[0] * i10];
    }

    emxFree_int32_T(&h_ii);
  }

  emxFree_boolean_T(&b_x);
  i10 = b_index->size[0] * b_index->size[1];
  b_index->size[0] = 1;
  b_index->size[1] = ii->size[1];
  emxEnsureCapacity((emxArray__common *)b_index, i10, (int)sizeof(double));
  nm1d2 = ii->size[0] * ii->size[1];
  for (i10 = 0; i10 < nm1d2; i10++) {
    b_index->data[i10] = ii->data[i10];
  }

  emxFree_int32_T(&ii);
  i10 = b_index->size[1];
  for (nm1d2 = 0; nm1d2 < i10; nm1d2++) {
    zmpUB_x->data[(int)b_index->data[nm1d2] - 1] = leftgait_x->data[(int)(offset
      + 1.0) - 1] + lfoot;
    zmpLB_x->data[(int)b_index->data[nm1d2] - 1] = leftgait_x->data[(int)(offset
      + 1.0) - 1];
    zmpUB_y->data[(int)b_index->data[nm1d2] - 1] = leftgait_y->data[(int)(offset
      + 1.0) - 1] + wfoot / 2.0;
    zmpLB_y->data[(int)b_index->data[nm1d2] - 1] = leftgait_y->data[(int)(offset
      + 1.0) - 1] - wfoot / 2.0;
  }

  emxFree_real_T(&b_index);

  /* line([leftgait_x(offset+1) + lfoot, leftgait_x(offset+1) + lfoot], [leftgait_y(offset+1) + wfoot/2, leftgait_y(offset+1) - wfoot/2]); */
  /* { */
  /* plot(x, zmpUB_y, 'r', x, zmpLB_y, 'r'); */
  /* xlabel('x(m)'); */
  /* ylabel('y(m)'); */
  /* set(gca,'ydir','reverse'); */
  /* hold off; */
  /*  */
  /* figure(3); */
  /* plot(t, zmpUB_x_t, t, zmpLB_x_t, 'g'); */
  /* title('ZMP-X Boundary Condition'); */
  /* xlabel('t(s)'); */
  /* ylabel('x(m)'); */
  /* } */
}

/* End of code generation (ZMP_Boundary_Condition.c) */
