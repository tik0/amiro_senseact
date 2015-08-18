//
// File: Odemetrie.h
//
// Code generated for Simulink model 'Odemetrie'.
//
// Model version                  : 1.329
// Simulink Coder version         : 8.7 (R2014b) 08-Sep-2014
// C/C++ source code generated on : Tue Aug 18 12:53:58 2015
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_Odemetrie_h_
#define RTW_HEADER_Odemetrie_h_
#include <math.h>
#include <stddef.h>
#ifndef Odemetrie_COMMON_INCLUDES_
# define Odemetrie_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 // Odemetrie_COMMON_INCLUDES_

#include "Odemetrie_types.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T UnitDelay4_DSTATE;            // '<S6>/Unit Delay4'
  real_T UnitDelay1_DSTATE;            // '<S6>/Unit Delay1'
  real_T UnitDelay2_DSTATE;            // '<S6>/Unit Delay2'
  real_T UnitDelay5_DSTATE;            // '<S6>/Unit Delay5'
  real_T UnitDelay6_DSTATE;            // '<S6>/Unit Delay6'
  real_T UnitDelay3_DSTATE;            // '<S6>/Unit Delay3'
} DW_Odemetrie_T;

// External inputs (root inport signals with auto storage)
typedef struct {
  real_T Machine_Speed;                // '<Root>/Machine_Speed'
  real_T Machine_EstimatedCurvature;   // '<Root>/Lenkwinkel'
  real_T Offsetcurvature;              // '<Root>/Offset curvature'
  real_T tSample;                      // '<Root>/tSample'
  real_T Diskrete_Auf;                 // '<Root>/Diskrete_Auf'
} ExtU_Odemetrie_T;

// External outputs (root outports fed by signals with auto storage)
typedef struct {
  real_T Phi_kon;                      // '<Root>/Phi_kon'
  real_T Y_dis_roi;                    // '<Root>/Y_dis_roi'
  real_T X_dis_roi;                    // '<Root>/X_dis_roi'
  real_T center_Dresch;                // '<Root>/center_Dresch'
  real_T X_dis_glo;                    // '<Root>/X_dis_glo'
  real_T Y_dis_glo;                    // '<Root>/Y_dis_glo'
  real_T X_kon_roi;                    // '<Root>/X_kon_roi'
  real_T X_kon_glo;                    // '<Root>/X_kon_glo'
  real_T Y_kon_roi;                    // '<Root>/Y_kon_roi'
  real_T Y_kon_glo;                    // '<Root>/Y_kon_glo'
} ExtY_Odemetrie_T;

// Parameters (auto storage)
struct P_Odemetrie_T_ {
  real_T Constant5_Value;              // Expression: 1
                                       //  Referenced by: '<S3>/Constant5'

  real_T Constant6_Value;              // Expression: -1
                                       //  Referenced by: '<S3>/Constant6'

  real_T _Value;                       // Expression: (1000/3600)
                                       //  Referenced by: '<S6>/3'

  real_T _Value_d;                     // Expression: 10^-3
                                       //  Referenced by: '<S6>/4'

  real_T UnitDelay4_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<S6>/Unit Delay4'

  real_T _Value_h;                     // Expression: 2
                                       //  Referenced by: '<S6>/2'

  real_T Gain1_Gain;                   // Expression: -1
                                       //  Referenced by: '<S6>/Gain1'

  real_T UnitDelay1_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<S6>/Unit Delay1'

  real_T UnitDelay2_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<S6>/Unit Delay2'

  real_T UnitDelay5_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<S6>/Unit Delay5'

  real_T UnitDelay6_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<S6>/Unit Delay6'

  real_T UnitDelay3_InitialCondition;  // Expression: 0
                                       //  Referenced by: '<S6>/Unit Delay3'

  real_T Gain1_Gain_l;                 // Expression: 360/(2*pi)
                                       //  Referenced by: '<S1>/Gain1'

  real_T Constant3_Value;              // Expression: 300
                                       //  Referenced by: '<S3>/Constant3'

  real_T Constant4_Value;              // Expression: 300
                                       //  Referenced by: '<S3>/Constant4'

  real_T Constant1_Value;              // Expression: 1
                                       //  Referenced by: '<S3>/Constant1'

  real_T Constant2_Value;              // Expression: 0
                                       //  Referenced by: '<S3>/Constant2'

  real_T Switch1_Threshold;            // Expression: 400
                                       //  Referenced by: '<S3>/Switch1'

  real_T Switch2_Threshold;            // Expression: 200
                                       //  Referenced by: '<S3>/Switch2'

  real_T Switch3_Threshold;            // Expression: 400
                                       //  Referenced by: '<S3>/Switch3'

  real_T Switch4_Threshold;            // Expression: 200
                                       //  Referenced by: '<S3>/Switch4'

  real_T _Value_c;                     // Expression: 0
                                       //  Referenced by: '<S6>/1'

  real_T Switch1_Threshold_e;          // Expression: 0
                                       //  Referenced by: '<S6>/Switch1'

  real_T Switch2_Threshold_i;          // Expression: 0
                                       //  Referenced by: '<S6>/Switch2'

};

// Real-time Model Data Structure
struct tag_RTM_Odemetrie_T {
  const char_T * volatile errorStatus;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// Class declaration for model Odemetrie
class OdemetrieModelClass {
  // public data and function members
 public:
  // External inputs
  ExtU_Odemetrie_T Odemetrie_U;

  // External outputs
  ExtY_Odemetrie_T Odemetrie_Y;

  // Model entry point functions

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  void terminate();

  // Constructor
  OdemetrieModelClass();

  // Destructor
  ~OdemetrieModelClass();

  // Real-Time Model get method
  RT_MODEL_Odemetrie_T * getRTM();

  // private data and function members
 private:
  // Tunable parameters
  P_Odemetrie_T Odemetrie_P;

  // Block states
  DW_Odemetrie_T Odemetrie_DW;

  // Real-Time Model
  RT_MODEL_Odemetrie_T Odemetrie_M;
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('gm_modell/Berechnungen/Datenverarbeitung/Odemetrie')    - opens subsystem gm_modell/Berechnungen/Datenverarbeitung/Odemetrie
//  hilite_system('gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'gm_modell/Berechnungen/Datenverarbeitung'
//  '<S1>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie'
//  '<S2>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Diskretisierung1'
//  '<S3>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Herausfahren aus der Bounding Box (neue Roi definieren)'
//  '<S4>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Model Info1'
//  '<S5>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Model Info2'
//  '<S6>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie'
//  '<S7>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Diskretisierung1/Model Info10'
//  '<S8>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Diskretisierung1/Model Info8'
//  '<S9>'   : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Diskretisierung1/Model Info9'
//  '<S10>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info'
//  '<S11>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info1'
//  '<S12>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info10'
//  '<S13>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info11'
//  '<S14>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info2'
//  '<S15>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info3'
//  '<S16>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info4'
//  '<S17>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info5'
//  '<S18>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info6'
//  '<S19>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info7'
//  '<S20>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info8'
//  '<S21>'  : 'gm_modell/Berechnungen/Datenverarbeitung/Odemetrie/Odemetrie/Model Info9'

#endif                                 // RTW_HEADER_Odemetrie_h_

//
// File trailer for generated code.
//
// [EOF]
//
