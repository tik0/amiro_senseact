
#ifndef SDK4_BINNING_CONTROL_H_
#define SDK4_BINNING_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>
#include <SDK4/SDK4FDC10Types.h>

#ifdef __cplusplus
extern "C" {
#endif
  
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetBinning(CTRL_HANDLE hCtrl,SDK4_KBINNING* kBinning);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetBinning(CTRL_HANDLE hCtrl,SDK4_KBINNING kBinning);

#ifdef __cplusplus
}
#endif

#endif
