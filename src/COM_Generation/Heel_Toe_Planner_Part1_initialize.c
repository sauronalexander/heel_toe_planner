/*
 * Heel_Toe_Planner_Part1_initialize.c
 *
 * Code generation for function 'Heel_Toe_Planner_Part1_initialize'
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
#include "heel_toe_planner/COM_Generation/Heel_Toe_Planner_Part1_initialize.h"
#include <stdio.h>


/* Function Definitions */
void Heel_Toe_Planner_Part1_initialize(void)
{
  rt_InitInfAndNaN(8U);
}

void Initial_emxArry(emxArray_real_T * array)
{
    array->allocatedSize = 0;
    array->canFreeData = true;
    array->size = calloc(2, sizeof(int));
    array->size[0] = 1;
    array->size[1] = 0;
    array->numDimensions = 2;
    array->data = NULL;

}


/* End of code generation (Heel_Toe_Planner_Part1_initialize.c) */
