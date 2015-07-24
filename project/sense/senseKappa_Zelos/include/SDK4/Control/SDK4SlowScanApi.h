
#ifndef SDK4_SLOWSCAN_CONTROL_H_
#define SDK4_SLOWSCAN_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif

	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetSlowScan(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH* puOnOff);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetSlowScan(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH uOnOff);

#ifdef __cplusplus
}
#endif

#endif
