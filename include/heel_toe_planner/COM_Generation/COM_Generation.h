/*
 * COM_Generation.h
 *
 * Code generation for function 'COM_Generation'
 *
 * C source code generated on: Wed Oct 19 16:30:45 2016
 *
 */

#ifndef __COM_GENERATION_H__
#define __COM_GENERATION_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "heel_toe_planner/COM_Generation/rt_nonfinite.h"

#include "heel_toe_planner/COM_Generation/rtwtypes.h"
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_types.h"

#define b_pi 3.14159265358979323846

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void test_COM_Generation();

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
                    emxArray_real_T *z_right);

#ifdef __cplusplus
}
#endif


#endif
/* End of code generation (COM_Generation.h) */
