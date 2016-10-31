/*
 * mldivide.c
 *
 * Code generation for function 'mldivide'
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
#include "heel_toe_planner/COM_Generation/mldivide.h"
#include <stdio.h>

/* Function Definitions */
void b_mldivide(const double A[25], double B[5])
{
  double b_A[25];
  signed char ipiv[5];
  int i13;
  int j;
  int c;
  int iy;
  int ix;
  double temp;
  int jy;
  double s;
  int b_j;
  int ijA;
  memcpy(&b_A[0], &A[0], 25U * sizeof(double));
  for (i13 = 0; i13 < 5; i13++) {
    ipiv[i13] = (signed char)(1 + i13);
  }

  for (j = 0; j < 4; j++) {
    c = j * 6;
    iy = 0;
    ix = c;
    temp = fabs(b_A[c]);
    for (jy = 2; jy <= 5 - j; jy++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > temp) {
        iy = jy - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (jy = 0; jy < 5; jy++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 5;
          iy += 5;
        }
      }

      i13 = (c - j) + 5;
      for (iy = c + 1; iy + 1 <= i13; iy++) {
        b_A[iy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 5;
    for (b_j = 1; b_j <= 4 - j; b_j++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0) {
        ix = c + 1;
        i13 = (iy - j) + 10;
        for (ijA = 6 + iy; ijA + 1 <= i13; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 5;
      iy += 5;
    }
  }

  for (iy = 0; iy < 5; iy++) {
    if (ipiv[iy] != iy + 1) {
      temp = B[iy];
      B[iy] = B[ipiv[iy] - 1];
      B[ipiv[iy] - 1] = temp;
    }
  }

  for (jy = 0; jy < 5; jy++) {
    c = 5 * jy;
    if (B[jy] != 0.0) {
      for (iy = jy + 2; iy < 6; iy++) {
        B[iy - 1] -= B[jy] * b_A[(iy + c) - 1];
      }
    }
  }

  for (jy = 4; jy > -1; jy += -1) {
    c = 5 * jy;
    if (B[jy] != 0.0) {
      B[jy] /= b_A[jy + c];
      for (iy = 0; iy + 1 <= jy; iy++) {
        B[iy] -= B[jy] * b_A[iy + c];
      }
    }
  }
}

void c_mldivide(const double A[16], double B[4])
{
  double b_A[16];
  signed char ipiv[4];
  int i14;
  int j;
  int c;
  int iy;
  int ix;
  double temp;
  int jy;
  double s;
  int b_j;
  int ijA;
  memcpy(&b_A[0], &A[0], sizeof(double) << 4);
  for (i14 = 0; i14 < 4; i14++) {
    ipiv[i14] = (signed char)(1 + i14);
  }

  for (j = 0; j < 3; j++) {
    c = j * 5;
    iy = 0;
    ix = c;
    temp = fabs(b_A[c]);
    for (jy = 2; jy <= 4 - j; jy++) {
      ix++;
      s = fabs(b_A[ix]);
      if (s > temp) {
        iy = jy - 1;
        temp = s;
      }
    }

    if (b_A[c + iy] != 0.0) {
      if (iy != 0) {
        ipiv[j] = (signed char)((j + iy) + 1);
        ix = j;
        iy += j;
        for (jy = 0; jy < 4; jy++) {
          temp = b_A[ix];
          b_A[ix] = b_A[iy];
          b_A[iy] = temp;
          ix += 4;
          iy += 4;
        }
      }

      i14 = (c - j) + 4;
      for (iy = c + 1; iy + 1 <= i14; iy++) {
        b_A[iy] /= b_A[c];
      }
    }

    iy = c;
    jy = c + 4;
    for (b_j = 1; b_j <= 3 - j; b_j++) {
      temp = b_A[jy];
      if (b_A[jy] != 0.0) {
        ix = c + 1;
        i14 = (iy - j) + 8;
        for (ijA = 5 + iy; ijA + 1 <= i14; ijA++) {
          b_A[ijA] += b_A[ix] * -temp;
          ix++;
        }
      }

      jy += 4;
      iy += 4;
    }
  }

  for (iy = 0; iy < 4; iy++) {
    if (ipiv[iy] != iy + 1) {
      temp = B[iy];
      B[iy] = B[ipiv[iy] - 1];
      B[ipiv[iy] - 1] = temp;
    }
  }

  for (jy = 0; jy < 4; jy++) {
    c = jy << 2;
    if (B[jy] != 0.0) {
      for (iy = jy + 2; iy < 5; iy++) {
        B[iy - 1] -= B[jy] * b_A[(iy + c) - 1];
      }
    }
  }

  for (jy = 3; jy > -1; jy += -1) {
    c = jy << 2;
    if (B[jy] != 0.0) {
      B[jy] /= b_A[jy + c];
      for (iy = 0; iy + 1 <= jy; iy++) {
        B[iy] -= B[jy] * b_A[iy + c];
      }
    }
  }
}

void mldivide(const double A[4], const double B[2], double Y[2])
{
  int r1;
  int r2;
  double a21;
  if (fabs(A[1]) > fabs(A[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = A[r2] / A[r1];
  Y[1] = (B[r2] - B[r1] * a21) / (A[2 + r2] - a21 * A[2 + r1]);
  Y[0] = (B[r1] - Y[1] * A[2 + r1]) / A[r1];
}



/* End of code generation (mldivide.c) */
