#ifndef __SDK4_INTERNALTESTS_CONTROL_H_
#define __SDK4_INTERNALTESTS_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif
	// only Zelos
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetPatternGenerator(CTRL_HANDLE hCtrl,SDK4_ENUM_PATTERN* puSelect);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetPatternGenerator(CTRL_HANDLE hCtrl,SDK4_ENUM_PATTERN uSelect);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetTemperature(CTRL_HANDLE hCtrl,int32_t uSelect,uint32_t* puTemp);

#ifdef __cplusplus
	}
#endif

#endif