/*
 * Auswertung_emxutil.h
 *
 * Code generation for function 'Auswertung_emxutil'
 *
 * C source code generated on: Wed Jul 15 11:14:12 2015
 *
 */

#ifndef __AUSWERTUNG_EMXUTIL_H__
#define __AUSWERTUNG_EMXUTIL_H__
/* Include files */
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "rtwtypes.h"
#include "Auswertung_types.h"

/* Function Declarations */
extern void emxEnsureCapacity(emxArray__common *emxArray, int oldNumel, int elementSize);
extern void emxFree_boolean_T(emxArray_boolean_T **pEmxArray);
extern void emxFree_int32_T(emxArray_int32_T **pEmxArray);
extern void emxFree_uint8_T(emxArray_uint8_T **pEmxArray);
extern void emxInit_boolean_T(emxArray_boolean_T **pEmxArray, int numDimensions);
extern void emxInit_int32_T(emxArray_int32_T **pEmxArray, int numDimensions);
extern void emxInit_uint8_T(emxArray_uint8_T **pEmxArray, int numDimensions);
#endif
/* End of code generation (Auswertung_emxutil.h) */
