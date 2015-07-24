
#ifndef SDK4_TRIGGER_CONTROL_H_
#define SDK4_TRIGGER_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif
	
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetSoftTrigger(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH* piOnOff);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetSoftTrigger(CTRL_HANDLE hCtrl,SDK4_ENUM_SWITCH iOnOff);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetTriggerTimer(CTRL_HANDLE hCtrl,uint32_t* puTime);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetTriggerTimer(CTRL_HANDLE hCtrl,uint32_t uTime);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetTriggerDelay(CTRL_HANDLE hCtrl,SDK4_KEXPOSURE* pkDelay);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetTriggerDelay(CTRL_HANDLE hCtrl,SDK4_KEXPOSURE kDelay);

#ifdef __cplusplus
}
#endif

#endif
