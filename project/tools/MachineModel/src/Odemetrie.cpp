/*
 * File: Odemetrie.cpp
 *
 * Code generated for Simulink model 'Odemetrie'.
 *
 * Model version                  : 1.323
 * Simulink Coder version         : 8.5 (R2013b) 08-Aug-2013
 * C/C++ source code generated on : Wed Jul 15 08:01:34 2015
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Odemetrie.h"
#include "Odemetrie_private.h"

/* Block states (auto storage) */
DW_Odemetrie_T Odemetrie_DW;

/* External inputs (root inport signals with auto storage) */
ExtU_Odemetrie_T Odemetrie_U;

/* External outputs (root outports fed by signals with auto storage) */
ExtY_Odemetrie_T Odemetrie_Y;

/* Real-time model */
RT_MODEL_Odemetrie_T Odemetrie_M_;
RT_MODEL_Odemetrie_T *const Odemetrie_M = &Odemetrie_M_;
real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function */
void Odemetrie_step(void)
{
  real_T rtb_Add3;
  real_T rtb_RoundingFunction2;
  real_T rtb_RoundingFunction1;
  real_T rtb_RoundingFunction;
  real_T rtb_Add1;
  boolean_T rtb_DataTypeConversion2;
  boolean_T rtb_DataTypeConversion3;
  boolean_T rtb_DataTypeConversion1;
  real_T rtb_Add4;
  real_T rtb_Add5;
  real_T rtb_Add6;

  /* Product: '<S6>/Product1' incorporates:
   *  Constant: '<S6>/3'
   *  Inport: '<Root>/Machine_Speed'
   */
  rtb_Add3 = Odemetrie_U.Machine_Speed * Odemetrie_P._Value;

  /* Product: '<S6>/Product4' incorporates:
   *  Inport: '<Root>/tSample'
   */
  rtb_RoundingFunction2 = Odemetrie_U.tSample * rtb_Add3;

  /* Product: '<S6>/Product3' incorporates:
   *  Constant: '<S6>/4'
   *  Inport: '<Root>/Lenkwinkel'
   *  Inport: '<Root>/Offset curvature'
   *  Product: '<S6>/Product2'
   *  Sum: '<S6>/Add'
   */
  rtb_Add3 *= Odemetrie_U.Machine_EstimatedCurvature * Odemetrie_P._Value_d -
    Odemetrie_U.Offsetcurvature;

  /* Product: '<S6>/Product5' incorporates:
   *  Inport: '<Root>/tSample'
   */
  rtb_RoundingFunction1 = Odemetrie_U.tSample * rtb_Add3;

  /* Sum: '<S6>/Add2' incorporates:
   *  Constant: '<S6>/2'
   *  Inport: '<Root>/tSample'
   *  Product: '<S6>/Divide'
   *  Product: '<S6>/Product6'
   *  UnitDelay: '<S6>/Unit Delay4'
   */
  rtb_Add3 = Odemetrie_U.tSample / Odemetrie_P._Value_h * rtb_Add3 +
    (rtb_RoundingFunction1 + Odemetrie_DW.UnitDelay4_DSTATE);

  /* Gain: '<S6>/Gain1' incorporates:
   *  Product: '<S6>/Product8'
   *  Trigonometry: '<S6>/Trigonometric Function1'
   */
  rtb_RoundingFunction = rtb_RoundingFunction2 * sin(rtb_Add3) *
    Odemetrie_P.Gain1_Gain;

  /* Sum: '<S6>/Add1' incorporates:
   *  UnitDelay: '<S6>/Unit Delay1'
   */
  rtb_Add1 = rtb_RoundingFunction + Odemetrie_DW.UnitDelay1_DSTATE;

  /* Product: '<S6>/Product7' incorporates:
   *  Trigonometry: '<S6>/Trigonometric Function2'
   */
  rtb_RoundingFunction2 *= cos(rtb_Add3);

  /* Sum: '<S6>/Add3' incorporates:
   *  UnitDelay: '<S6>/Unit Delay2'
   */
  rtb_Add3 = rtb_RoundingFunction2 + Odemetrie_DW.UnitDelay2_DSTATE;

  /* Sum: '<S6>/Add4' incorporates:
   *  UnitDelay: '<S6>/Unit Delay3'
   */
  rtb_Add4 = rtb_RoundingFunction1 + Odemetrie_DW.UnitDelay3_DSTATE;

  /* Sum: '<S6>/Add5' incorporates:
   *  UnitDelay: '<S6>/Unit Delay5'
   */
  rtb_Add5 = rtb_RoundingFunction + Odemetrie_DW.UnitDelay5_DSTATE;

  /* Sum: '<S6>/Add6' incorporates:
   *  UnitDelay: '<S6>/Unit Delay6'
   */
  rtb_Add6 = rtb_RoundingFunction2 + Odemetrie_DW.UnitDelay6_DSTATE;

  /* Rounding: '<S2>/Rounding Function' incorporates:
   *  Inport: '<Root>/Diskrete_Auf'
   *  Product: '<S2>/Divide'
   */
  rtb_RoundingFunction = rt_roundd_snf(rtb_Add1 / Odemetrie_U.Diskrete_Auf);

  /* Sum: '<S3>/Add1' incorporates:
   *  Constant: '<S3>/Constant4'
   */
  rtb_RoundingFunction1 = rtb_RoundingFunction + Odemetrie_P.Constant4_Value;

  /* Switch: '<S3>/Switch1' */
  if (rtb_RoundingFunction1 >= Odemetrie_P.Switch1_Threshold) {
    /* DataTypeConversion: '<S3>/Data Type Conversion2' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    rtb_DataTypeConversion2 = (Odemetrie_P.Constant1_Value != 0.0);
  } else {
    /* DataTypeConversion: '<S3>/Data Type Conversion2' incorporates:
     *  Constant: '<S3>/Constant2'
     */
    rtb_DataTypeConversion2 = (Odemetrie_P.Constant2_Value != 0.0);
  }

  /* End of Switch: '<S3>/Switch1' */

  /* Switch: '<S3>/Switch2' incorporates:
   *  Constant: '<S3>/Constant1'
   *  Constant: '<S3>/Constant2'
   */
  if (rtb_RoundingFunction1 >= Odemetrie_P.Switch2_Threshold) {
    rtb_RoundingFunction1 = Odemetrie_P.Constant2_Value;
  } else {
    rtb_RoundingFunction1 = Odemetrie_P.Constant1_Value;
  }

  /* End of Switch: '<S3>/Switch2' */

  /* DataTypeConversion: '<S3>/Data Type Conversion3' */
  rtb_DataTypeConversion3 = (rtb_RoundingFunction1 != 0.0);

  /* Rounding: '<S2>/Rounding Function1' incorporates:
   *  Inport: '<Root>/Diskrete_Auf'
   *  Product: '<S2>/Divide1'
   */
  rtb_RoundingFunction1 = rt_roundd_snf(rtb_Add3 / Odemetrie_U.Diskrete_Auf);

  /* Sum: '<S3>/Add' incorporates:
   *  Constant: '<S3>/Constant3'
   */
  rtb_RoundingFunction2 = rtb_RoundingFunction1 + Odemetrie_P.Constant3_Value;

  /* Switch: '<S3>/Switch3' */
  if (rtb_RoundingFunction2 >= Odemetrie_P.Switch3_Threshold) {
    /* DataTypeConversion: '<S3>/Data Type Conversion1' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    rtb_DataTypeConversion1 = (Odemetrie_P.Constant1_Value != 0.0);
  } else {
    /* DataTypeConversion: '<S3>/Data Type Conversion1' incorporates:
     *  Constant: '<S3>/Constant2'
     */
    rtb_DataTypeConversion1 = (Odemetrie_P.Constant2_Value != 0.0);
  }

  /* End of Switch: '<S3>/Switch3' */

  /* Switch: '<S3>/Switch4' incorporates:
   *  Constant: '<S3>/Constant1'
   *  Constant: '<S3>/Constant2'
   */
  if (rtb_RoundingFunction2 >= Odemetrie_P.Switch4_Threshold) {
    rtb_RoundingFunction2 = Odemetrie_P.Constant2_Value;
  } else {
    rtb_RoundingFunction2 = Odemetrie_P.Constant1_Value;
  }

  /* End of Switch: '<S3>/Switch4' */

  /* Switch: '<S3>/Switch5' incorporates:
   *  Constant: '<S3>/Constant5'
   *  Constant: '<S3>/Constant6'
   *  DataTypeConversion: '<S3>/Data Type Conversion'
   *  S-Function (sfix_bitop): '<S3>/Bitwise Operator'
   */
  if ((boolean_T)((boolean_T)((boolean_T)(rtb_DataTypeConversion2 |
         rtb_DataTypeConversion3) | rtb_DataTypeConversion1) |
                  (rtb_RoundingFunction2 != 0.0))) {
    rtb_RoundingFunction2 = Odemetrie_P.Constant6_Value;
  } else {
    rtb_RoundingFunction2 = Odemetrie_P.Constant5_Value;
  }

  /* End of Switch: '<S3>/Switch5' */

  /* Outport: '<Root>/Phi_kon' incorporates:
   *  Gain: '<S1>/Gain1'
   */
  Odemetrie_Y.Phi_kon = Odemetrie_P.Gain1_Gain_l * rtb_Add4;

  /* Outport: '<Root>/Y_dis_roi' */
  Odemetrie_Y.Y_dis_roi = rtb_RoundingFunction1;

  /* Outport: '<Root>/X_dis_roi' */
  Odemetrie_Y.X_dis_roi = rtb_RoundingFunction;

  /* Outport: '<Root>/center_Dresch' */
  Odemetrie_Y.center_Dresch = rtb_RoundingFunction2;

  /* Outport: '<Root>/X_dis_glo' incorporates:
   *  Inport: '<Root>/Diskrete_Auf'
   *  Product: '<S2>/Divide2'
   *  Rounding: '<S2>/Rounding Function2'
   */
  Odemetrie_Y.X_dis_glo = rt_roundd_snf(rtb_Add5 / Odemetrie_U.Diskrete_Auf);

  /* Outport: '<Root>/Y_dis_glo' incorporates:
   *  Inport: '<Root>/Diskrete_Auf'
   *  Product: '<S2>/Divide3'
   *  Rounding: '<S2>/Rounding Function3'
   */
  Odemetrie_Y.Y_dis_glo = rt_roundd_snf(rtb_Add6 / Odemetrie_U.Diskrete_Auf);

  /* Outport: '<Root>/X_kon_roi' */
  Odemetrie_Y.X_kon_roi = rtb_Add1;

  /* Outport: '<Root>/X_kon_glo' */
  Odemetrie_Y.X_kon_glo = rtb_Add5;

  /* Outport: '<Root>/Y_kon_roi' */
  Odemetrie_Y.Y_kon_roi = rtb_Add3;

  /* Outport: '<Root>/Y_kon_glo' */
  Odemetrie_Y.Y_kon_glo = rtb_Add6;

  /* Update for UnitDelay: '<S6>/Unit Delay4' */
  Odemetrie_DW.UnitDelay4_DSTATE = rtb_Add4;

  /* Switch: '<S6>/Switch2' */
  if (rtb_RoundingFunction2 >= Odemetrie_P.Switch2_Threshold_i) {
    /* Update for UnitDelay: '<S6>/Unit Delay1' */
    Odemetrie_DW.UnitDelay1_DSTATE = rtb_Add1;
  } else {
    /* Update for UnitDelay: '<S6>/Unit Delay1' incorporates:
     *  Constant: '<S6>/1'
     */
    Odemetrie_DW.UnitDelay1_DSTATE = Odemetrie_P._Value_c;
  }

  /* End of Switch: '<S6>/Switch2' */

  /* Switch: '<S6>/Switch1' */
  if (rtb_RoundingFunction2 >= Odemetrie_P.Switch1_Threshold_e) {
    /* Update for UnitDelay: '<S6>/Unit Delay2' */
    Odemetrie_DW.UnitDelay2_DSTATE = rtb_Add3;
  } else {
    /* Update for UnitDelay: '<S6>/Unit Delay2' incorporates:
     *  Constant: '<S6>/1'
     */
    Odemetrie_DW.UnitDelay2_DSTATE = Odemetrie_P._Value_c;
  }

  /* End of Switch: '<S6>/Switch1' */

  /* Update for UnitDelay: '<S6>/Unit Delay3' */
  Odemetrie_DW.UnitDelay3_DSTATE = rtb_Add4;

  /* Update for UnitDelay: '<S6>/Unit Delay5' */
  Odemetrie_DW.UnitDelay5_DSTATE = rtb_Add5;

  /* Update for UnitDelay: '<S6>/Unit Delay6' */
  Odemetrie_DW.UnitDelay6_DSTATE = rtb_Add6;
}

/* Model initialize function */
void Odemetrie_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(Odemetrie_M, (NULL));

  /* states (dwork) */
  Odemetrie_DW.UnitDelay4_DSTATE = 0.0;
  Odemetrie_DW.UnitDelay1_DSTATE = 0.0;
  Odemetrie_DW.UnitDelay2_DSTATE = 0.0;
  Odemetrie_DW.UnitDelay3_DSTATE = 0.0;
  Odemetrie_DW.UnitDelay5_DSTATE = 0.0;
  Odemetrie_DW.UnitDelay6_DSTATE = 0.0;

  /* external inputs */
  Odemetrie_U.Machine_Speed = 0.0;
  Odemetrie_U.Machine_EstimatedCurvature = 0.0;
  Odemetrie_U.Offsetcurvature = 0.0;
  Odemetrie_U.tSample = 0.0;
  Odemetrie_U.Diskrete_Auf = 0.0;

  /* external outputs */
  Odemetrie_Y.Phi_kon = 0.0;
  Odemetrie_Y.Y_dis_roi = 0.0;
  Odemetrie_Y.X_dis_roi = 0.0;
  Odemetrie_Y.center_Dresch = 0.0;
  Odemetrie_Y.X_dis_glo = 0.0;
  Odemetrie_Y.Y_dis_glo = 0.0;
  Odemetrie_Y.X_kon_roi = 0.0;
  Odemetrie_Y.X_kon_glo = 0.0;
  Odemetrie_Y.Y_kon_roi = 0.0;
  Odemetrie_Y.Y_kon_glo = 0.0;

  /* InitializeConditions for UnitDelay: '<S6>/Unit Delay4' */
  Odemetrie_DW.UnitDelay4_DSTATE = Odemetrie_P.UnitDelay4_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S6>/Unit Delay1' */
  Odemetrie_DW.UnitDelay1_DSTATE = Odemetrie_P.UnitDelay1_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S6>/Unit Delay2' */
  Odemetrie_DW.UnitDelay2_DSTATE = Odemetrie_P.UnitDelay2_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S6>/Unit Delay3' */
  Odemetrie_DW.UnitDelay3_DSTATE = Odemetrie_P.UnitDelay3_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S6>/Unit Delay5' */
  Odemetrie_DW.UnitDelay5_DSTATE = Odemetrie_P.UnitDelay5_InitialCondition;

  /* InitializeConditions for UnitDelay: '<S6>/Unit Delay6' */
  Odemetrie_DW.UnitDelay6_DSTATE = Odemetrie_P.UnitDelay6_InitialCondition;
}

/* Model terminate function */
void Odemetrie_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
