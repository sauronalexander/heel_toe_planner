/*
 * fprintf.c
 *
 * Code generation for function 'fprintf'
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
#include "heel_toe_planner/COM_Generation/fprintf.h"
#include "heel_toe_planner/COM_Generation/fileManager.h"
#include <stdio.h>

/* Function Declarations */
static double c_fprintf(void);

/* Function Definitions */
static double c_fprintf(void)
{
  int nbytesint;
  FILE * b_NULL;
  boolean_T autoflush;
  FILE * filestar;
  static const char cfmt[38] = { 'E', 'R', 'R', 'O', 'R', ' ', 'i', 'n', ' ',
    'C', 'o', 's', 'i', 'n', 'e', '_', 'C', 'o', 's', 'i', 'n', 'e', '_', 'I',
    'n', 't', 'e', 'r', 'p', 'o', 'l', 'a', 't', 'i', 'o', 'n', '\x0a', '\x00' };

  nbytesint = 0;
  b_NULL = NULL;
  fileManager(&filestar, &autoflush);
  if (filestar == b_NULL) {
  } else {
    nbytesint = fprintf(filestar, "%s", cfmt);
    printf("%s\n", cfmt);
    fflush(filestar);
  }

  return nbytesint;
}

void b_fprintf(void)
{
  c_fprintf();
}

/* End of code generation (fprintf.c) */
