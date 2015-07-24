
#ifndef SDK4_GPIO_CONTROL_H_
#define SDK4_GPIO_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>
#include <SDK4/SDK4FDC10Types.h>

#ifdef __cplusplus
extern "C" {
#endif

	//Select ??? enum 
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetIoMode(CTRL_HANDLE hCtrl,uint32_t uSelect,SDK4_ENUM_IOMODE* puMode);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetIoMode(CTRL_HANDLE hCtrl,uint32_t uSelect, SDK4_ENUM_IOMODE uMode);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetIoPort(CTRL_HANDLE hCtrl,uint32_t uSelect,SDK4_ENUM_IOPORT* puPort);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetIoPort(CTRL_HANDLE hCtrl,uint32_t uSelect, SDK4_ENUM_IOPORT uPort);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetIoPolarity(CTRL_HANDLE hCtrl,uint32_t uSelect,SDK4_ENUM_IOPOLARITY* puPolarity);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetIoPolarity(CTRL_HANDLE hCtrl,uint32_t uSelect, SDK4_ENUM_IOPOLARITY uPolarity);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4GetIoDelay(CTRL_HANDLE hCtrl,uint32_t uSelect,uint32_t* puDelay);
	EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE  SDK4SetIoDelay(CTRL_HANDLE hCtrl,uint32_t uSelect, uint32_t uDelay);

#ifdef __cplusplus
}
#endif

#endif
