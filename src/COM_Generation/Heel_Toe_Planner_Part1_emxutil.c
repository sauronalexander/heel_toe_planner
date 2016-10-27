/*
 * Heel_Toe_Planner_Part1_emxutil.c
 *
 * Code generation for function 'Heel_Toe_Planner_Part1_emxutil'
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
#include <assert.h>

/* Function Definitions */
void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int elementSize)
{
  assert(emxArray->size[0]>=0);
  assert(emxArray->size[1]>=0);
  int newNumel;
  int i;
  void *newData;
  newNumel = 1;
  for (i = 0; i < emxArray->numDimensions; i++) {
    newNumel *= emxArray->size[i];
  }

  if (newNumel > emxArray->allocatedSize) {
    i = emxArray->allocatedSize;
    if (i < 16) {
      i = 16;
    }

    while (i < newNumel) {
      i <<= 1;
    }

    newData = calloc((unsigned int)i, (unsigned int)elementSize);
    if (emxArray->data != NULL) {
      memcpy(newData, emxArray->data, (unsigned int)(elementSize * oldNumel));
      if (emxArray->canFreeData) {
        free(emxArray->data);
      }
    }

    emxArray->data = newData;
    emxArray->allocatedSize = i;
    emxArray->canFreeData = TRUE;
  }
}

void emxFree_boolean_T(emxArray_boolean_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_boolean_T *)NULL) {
    if ((*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_boolean_T *)NULL;
  }
}

void emxFree_int32_T(emxArray_int32_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_int32_T *)NULL) {
    if ((*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_int32_T *)NULL;
  }
}

void emxFree_real_T(emxArray_real_T **pEmxArray)
{
  if (*pEmxArray != (emxArray_real_T *)NULL) {
    if ((*pEmxArray)->canFreeData) {
      free((void *)(*pEmxArray)->data);
    }

    free((void *)(*pEmxArray)->size);
    free((void *)*pEmxArray);
    *pEmxArray = (emxArray_real_T *)NULL;
  }
}

void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions)
{
  emxArray_boolean_T *emxArray;
  int i;
  *pEmxArray = (emxArray_boolean_T *)malloc(sizeof(emxArray_boolean_T));
  emxArray = *pEmxArray;
  emxArray->data = (boolean_T *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = TRUE;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions)
{
  emxArray_int32_T *emxArray;
  int i;
  *pEmxArray = (emxArray_int32_T *)malloc(sizeof(emxArray_int32_T));
  emxArray = *pEmxArray;
  emxArray->data = (int *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = TRUE;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

void emxInit_real_T(emxArray_real_T **pEmxArray, int numDimensions)
{
  emxArray_real_T *emxArray;
  int i;
  *pEmxArray = (emxArray_real_T *)malloc(sizeof(emxArray_real_T));
  emxArray = *pEmxArray;
  emxArray->data = (double *)NULL;
  emxArray->numDimensions = numDimensions;
  emxArray->size = (int *)malloc((unsigned int)(sizeof(int) * numDimensions));
  emxArray->allocatedSize = 0;
  emxArray->canFreeData = TRUE;
  for (i = 0; i < numDimensions; i++) {
    emxArray->size[i] = 0;
  }
}

/* End of code generation (Heel_Toe_Planner_Part1_emxutil.c) */
