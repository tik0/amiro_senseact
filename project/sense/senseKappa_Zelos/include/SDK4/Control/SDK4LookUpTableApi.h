
#ifndef SDK4_LOOKUPTABLE_CONTROL_H_
#define SDK4_LOOKUPTABLE_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>
#include <SDK4/SDK4FDC10Types.h>

#ifdef __cplusplus
extern "C" {
#endif

  //ILookUpTable
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetLUTInterpolatorIndex(CTRL_HANDLE hCtrl,int32_t iIndex);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetLUTInterpolatorIndex(CTRL_HANDLE hCtrl,int32_t* piIndex);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetLUTMode(CTRL_HANDLE hCtrl,SDK4_ENUM_LUTMODE eMode);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetLUTMode(CTRL_HANDLE hCtrl,SDK4_ENUM_LUTMODE* peMode);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetNegativeImage(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH uNegativeImage);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetNegativeImage(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH* puNegativeImage);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4LoadLUT(CTRL_HANDLE hCtrl,const char* filename);

#ifdef __cplusplus
}
#endif

#endif
