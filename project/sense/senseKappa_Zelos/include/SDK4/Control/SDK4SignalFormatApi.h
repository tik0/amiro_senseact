
#ifndef SDK4_SIGNALFORMAT_CONTROL_H_
#define SDK4_SIGNALFORMAT_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif

	//ISignalFormat
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetColorCoding(CTRL_HANDLE hCtrl,SDK4_ENUM_COLORCODING iSelect);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetColorCoding(CTRL_HANDLE hCtrl,SDK4_ENUM_COLORCODING* piSelect);

#ifdef __cplusplus
}
#endif

#endif
