/*
 * Gait_Generation.c
 *
 * Code generation for function 'Gait_Generation'
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
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_emxutil.h"
#include <stdio.h>

void test_gait() 
{ 
  printf("Testing Gait Generation\n");
}

/* Function Definitions */
void Gait_Generation(double totaltime, double tinit, double tend, double tstep,
                     double ratio, double Period, double lstep, double unusedU0,
                     double w, double unusedU1, emxArray_real_T *leftgait_x,
                     emxArray_real_T *rightgait_x, emxArray_real_T *leftgait_y,
                     emxArray_real_T *rightgait_y)
{

  double N;
  double x;
  double Sample_step;
  double Sample_init_SS;
  double Nsample;
  double tss;
  double Sample_init;
  int i6;
  int loop_ub;
  int j;
  double d0;
  double b_j;
  (void)unusedU0;
  (void)unusedU1;
//  if(leftgait_x->size == NULL)
//      leftgait_x->size = calloc(2, sizeof(int));
//  if(rightgait_x->size == NULL)
//      rightgait_x->size = calloc(2, sizeof(int));
//  if(leftgait_y->size == NULL)
//      leftgait_y->size = calloc(2, sizeof(int));
//  if(rightgait_y->size == NULL)
//      rightgait_y->size = calloc(2, sizeof(int));
//  leftgait_x->size[0] = 1;
//  leftgait_x->size[1] = 0;
//  rightgait_x->size[0] = 1;
//  rightgait_x->size[1] = 0;
//  leftgait_y->size[0] = 1;
//  leftgait_y->size[1] = 0;
//  rightgait_y->size[0] = 1;
//  rightgait_y->size[1] = 0;
  /* %Gait Generation */
  /* input: totaltime, tinit, tend, tstep, ratio, Period(for sample), lstep, lfoot, w(hipwidth), wfoot; */
  /* output: leftgait_x, rightgait_x, leftgait_y, rightgait_y; */
  /* General Initialization */
  N = ((totaltime - tinit) - tend) / tstep;
  x = floor(N);
  N = floor(N) + 2.0;
  /* Number of steps */
  Sample_step = floor(tstep / Period);
  Sample_init_SS = floor(10.0 * tinit / (10.0 * Period));
  Nsample = (((x + 2.0) - 2.0) * Sample_step + Sample_init_SS) + (floor(tend /
    Period) + 1.0);
printf("%f\n", Nsample);
  /* Number of samples  */
  tss = (1.0 - ratio) * tstep;
  Sample_init = Sample_init_SS + floor(ratio * tstep / Period);
  /* %initialization */
  i6 = leftgait_x->size[0] * leftgait_x->size[1];
  leftgait_x->size[0] = 1;
  leftgait_x->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)leftgait_x, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    leftgait_x->data[i6] = 0.0;
  }

  i6 = leftgait_y->size[0] * leftgait_y->size[1];
  leftgait_y->size[0] = 1;
  leftgait_y->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)leftgait_y, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    leftgait_y->data[i6] = 0.0;
  }

  i6 = rightgait_x->size[0] * rightgait_x->size[1];
  rightgait_x->size[0] = 1;
  rightgait_x->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)rightgait_x, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    rightgait_x->data[i6] = 0.0;
  }

  i6 = rightgait_y->size[0] * rightgait_y->size[1];
  rightgait_y->size[0] = 1;
  rightgait_y->size[1] = (int)Nsample;
  emxEnsureCapacity((emxArray__common *)rightgait_y, i6, (int)sizeof(double));
  loop_ub = (int)Nsample;
  for (i6 = 0; i6 < loop_ub; i6++) {
    rightgait_y->data[i6] = 0.0;
  }

  for (loop_ub = 0; loop_ub < (int)N; loop_ub++) {
    if (1.0 + (double)loop_ub == 1.0) {
      i6 = (int)(Nsample + (1.0 - (Sample_init_SS + 1.0)));
      for (j = 0; j < i6; j++) {
        leftgait_x->data[(int)((Sample_init_SS + 1.0) + (double)j) - 1] = lstep;
      }
    } else if (1.0 + (double)loop_ub == x + 2.0) {
      rightgait_x->data[(int)Nsample - 1] += lstep;
    } else if ((1.0 + (double)loop_ub) - floor((1.0 + (double)loop_ub) / 2.0) *
               2.0 == 0.0) {
      d0 = ((((1.0 + (double)loop_ub) - 2.0) * Sample_step + 1.0) + Sample_init)
        + floor(tss / Period);
      i6 = (int)(Nsample + (1.0 - d0));
      for (j = 0; j < i6; j++) {
        b_j = d0 + (double)j;
        rightgait_x->data[(int)b_j - 1] += 2.0 * lstep;
      }
    } else {
      d0 = ((((1.0 + (double)loop_ub) - 2.0) * Sample_step + 1.0) + Sample_init)
        + floor(tss / Period);
      i6 = (int)(Nsample + (1.0 - d0));
      for (j = 0; j < i6; j++) {
        b_j = d0 + (double)j;
        leftgait_x->data[(int)b_j - 1] += 2.0 * lstep;
      }
    }
  }

  for (loop_ub = 0; loop_ub < (int)Nsample; loop_ub++) {
    leftgait_y->data[loop_ub] = -w / 2.0;
    rightgait_y->data[loop_ub] = w / 2.0;
  }

  /* {  */
  /* figure(1); */
  /* subplot(2, 1, 1); */
  /* plot(t, leftgait_x, t, rightgait_x, 'g'); */
  /* ylabel('x(m)'); */
  /* xlabel('t(s)'); */
  /* legend('Left Gait', 'Right Gait'); */
  /* title('Gait Generation--X'); */
  /*  */
  /* subplot(2, 1, 2); */
  /* plot(t, leftgait_y, t, rightgait_y, 'g'); */
  /* ylabel('y(m)'); */
  /* xlabel('t(s)'); */
  /* legend('Left Gait', 'Right Gait'); */
  /* title('Gait Generation--Y'); */
  /* } */
}

/* End of code generation (Gait_Generation.c) */
