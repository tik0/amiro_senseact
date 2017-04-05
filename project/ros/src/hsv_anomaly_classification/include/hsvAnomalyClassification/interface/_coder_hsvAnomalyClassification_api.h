/*
 * File: _coder_hsvAnomalyClassification_api.h
 *
 * MATLAB Coder version            : 3.1
 * C/C++ source code generated on  : 05-Apr-2017 07:49:52
 */

#ifndef _CODER_HSVANOMALYCLASSIFICATION_API_H
#define _CODER_HSVANOMALYCLASSIFICATION_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_hsvAnomalyClassification_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void hsvAnomalyClassification(real_T x1[1920000], real_T b_y1[640000]);
extern void hsvAnomalyClassification_api(const mxArray *prhs[1], const mxArray
  *plhs[1]);
extern void hsvAnomalyClassification_atexit(void);
extern void hsvAnomalyClassification_initialize(void);
extern void hsvAnomalyClassification_terminate(void);
extern void hsvAnomalyClassification_xil_terminate(void);

#endif

/*
 * File trailer for _coder_hsvAnomalyClassification_api.h
 *
 * [EOF]
 */
