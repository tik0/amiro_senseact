
#ifndef SDK4_GIGEVISION_CONTROL_H_
#define SDK4_GIGEVISION_CONTROL_H_

#include <SDK4/SDK4PlatformTypes.h>
#include <SDK4/SDK4AcquireTypes.h>
#include <SDK4/SDK4CommonControlTypes.h>

#ifdef __cplusplus
extern "C" {
#endif
  //IGigECamera
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetGigEVersion(CTRL_HANDLE hCtrl,uint32_t *puGigEVersion);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetMacAddress(CTRL_HANDLE hCtrl,char * psMacAddress, int32_t *piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetCurrentIPAddress(CTRL_HANDLE hCtrl,char* psIPAddress,int32_t *piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetCurrentSubnetmask(CTRL_HANDLE hCtrl,char* psSubnetmask,int32_t *piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetCurrentDefaultGateway(CTRL_HANDLE hCtrl,char* psDefaultGateway,int32_t *piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetPacketSize(CTRL_HANDLE hCtrl,uint32_t uPacketSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetPacketSize(CTRL_HANDLE hCtrl,uint32_t *puPacketSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetUserDefinedName(CTRL_HANDLE hCtrl, char* psUserDefinedName, int32_t *piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetUserDefinedName(CTRL_HANDLE hCtrl, const char* psUserDefinedName, int32_t *piSize);


  /*
  //Unsupported
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetCurrentIPConfig(CTRL_HANDLE hCtrl,SDK4_FLAGS_IPCONFIG *puConfig);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4EnableStatic(CTRL_HANDLE hCtrl,const char * psIPAddress,const char * psSubnetMask,const char * psDefaultGateway);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DisableStatic(CTRL_HANDLE hCtrl);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4EnableDHCP(CTRL_HANDLE hCtrl);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DisableDHCP(CTRL_HANDLE hCtrl);
*/
#ifdef __cplusplus
}
#endif

#endif
