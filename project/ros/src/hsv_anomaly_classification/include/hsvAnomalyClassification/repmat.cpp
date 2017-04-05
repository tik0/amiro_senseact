//
// File: repmat.cpp
//
// MATLAB Coder version            : 3.1
// C/C++ source code generated on  : 05-Apr-2017 07:49:52
//

// Include Files
#include "rt_nonfinite.h"
#include "hsvAnomalyClassification.h"
#include "repmat.h"

// Function Definitions

//
// Arguments    : double b[3200000]
// Return Type  : void
//
void repmat(double b[3200000])
{
  int jtilecol;
  int ibtile;
  int k;
  static const double a[5] = { 10.792961920608143, -5.7545988164182713,
    4.2737419862546133, -3.4767271386545939, 5.3080821132311362 };

  for (jtilecol = 0; jtilecol < 640000; jtilecol++) {
    ibtile = jtilecol * 5;
    for (k = 0; k < 5; k++) {
      b[ibtile + k] = a[k];
    }
  }
}

//
// File trailer for repmat.cpp
//
// [EOF]
//
