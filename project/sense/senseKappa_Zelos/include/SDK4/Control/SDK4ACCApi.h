
#ifndef SDK4_ACC_CONTROL_H_
#define SDK4_ACC_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif


  //IACC
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetACC(CTRL_HANDLE hCtrl,SDK4_ENUM_ACC iSelect);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetACC(CTRL_HANDLE hCtrl,SDK4_ENUM_ACC* piSelect);
  
#ifdef __cplusplus
}
#endif

#endif
