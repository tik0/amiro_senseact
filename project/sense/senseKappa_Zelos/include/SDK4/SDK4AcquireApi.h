//version 1.2

#ifndef SDK4_BASE_API_H_
#define SDK4_BASE_API_H_

#include <SDK4/SDK4AcquireTypes.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) || defined(WIN32)
  #ifdef SDK4BASEAPI_EXPORTS
    #define SDK4_IMPORT_EXPORT __declspec(dllexport) 
  #else
    #define SDK4_IMPORT_EXPORT 
  #endif
  #define SDK4_CALLTYPE __stdcall  
  #if ! defined EXTERN_C
    #define EXTERN_C extern "C"
  #endif
#else

  #define SDK4_IMPORT_EXPORT 
  #define SDK4_CALLTYPE   
  #if ! defined EXTERN_C
    #define EXTERN_C extern "C"
  #endif

#endif

  typedef void * DEV_HANDLE;	
  typedef void * CTRL_HANDLE;	
  typedef void * DS_HANDLE;	
  typedef void * BUFFER_HANDLE;

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4InitLib ( void );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4CloseLib ( void );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetNumDevices ( int32_t* piNumDevices );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4GetDeviceID (int32_t iIndex, char * sDeviceID, int32_t *piSize  );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4OpenDevice (const char * psDeviceID, ENUM_DEVICE_ACCESS iOpenFlags, DEV_HANDLE* hDev );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4CloseDevice (DEV_HANDLE hDev );
  
  //Device Functions
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DevStartAcquisition	(DEV_HANDLE hDev );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DevStopAcquisition		(DEV_HANDLE hDev );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DevGetControl			(DEV_HANDLE hDev, CTRL_HANDLE* phCtrl);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DevGetNumDataStreams	(DEV_HANDLE hDev,int32_t* piNumDataStreams);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DevGetDataStream		(DEV_HANDLE hDev,int32_t iChannel, DS_HANDLE* phDataStream );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DevGetClassID			(DEV_HANDLE hDev ,char * sID, int32_t *pSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DevGetModel			(DEV_HANDLE hDev ,char * sModel, int32_t *pSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DevGetAccessStatus		(DEV_HANDLE hDev, ENUM_DEVICE_ACCESS_STATUS* pAccessStatus );
 
  //Grabber Functions
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSGetPayloadSize			(DS_HANDLE hDataStream, uint32_t* piSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSAnnounceBuffer			(DS_HANDLE hDataStream, void *pBuffer, size_t iSize, void *pPrivate, BUFFER_HANDLE *phBuffer);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSAllocAndAnnounceBuffer	(DS_HANDLE hDataStream, size_t iSize, void *pPrivate, BUFFER_HANDLE *phBuffer );
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSQueueBuffer				(DS_HANDLE hDataStream, BUFFER_HANDLE hBuffer);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSWaitForBuffer			(DS_HANDLE hDataStream, BUFFER_HANDLE* phBuffer, uint32_t timeout);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSFlushQueue				(DS_HANDLE hDataStream);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSRevokeBuffer				(DS_HANDLE hDataStream, BUFFER_HANDLE hBuffer, void **pBuffer, void **pPrivate);
  
  //VideoFormat Functions
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSGetWidth			(DS_HANDLE hDataStream, int32_t* pWidth);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSGetHeight		(DS_HANDLE hDataStream, int32_t* pHeight);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSGetPixelFormat	(DS_HANDLE hDataStream, ENUM_PIXELFORMAT* pFormat);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4DSGetTimePerFrame	(DS_HANDLE hDataStream, int32_t* pTimePerFrame);

  //Buffer Functions
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferGetPtr		(BUFFER_HANDLE hBuffer ,void** ppPtr);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferGetPrivatePtr(BUFFER_HANDLE hBuffer ,void** ppPrivatePtr);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferGetSize		(BUFFER_HANDLE hBuffer ,uint32_t* pSize);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferIsComplete	(BUFFER_HANDLE hBuffer ,int32_t* pIsComplete);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferGetFrameID   (BUFFER_HANDLE hBuffer ,uint32_t* pNumber);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferGetChunkData(BUFFER_HANDLE hBuffer, ENUM_CHUNKDATA_CMD iCmd, uint32_t* pData);

  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferSignalStatus(BUFFER_HANDLE hBuffer, uint32_t* pStatus);
  
  //Unsupported
  //EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferSizeFilled	 (BUFFER_HANDLE hBuffer ,uint64_t*  pSizeFilled);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferGetWidth		 (BUFFER_HANDLE hBuffer ,uint32_t* pWidth);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferGetHeight	 (BUFFER_HANDLE hBuffer ,uint32_t* pHeight);
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4BufferGetPixelFormat(BUFFER_HANDLE hBuffer ,uint32_t* pPixelType);
  //new 11062
  EXTERN_C SDK4_IMPORT_EXPORT  SDK4_ERROR  SDK4_CALLTYPE SDK4SetLogLevel(int32_t level);

#ifdef __cplusplus
}
#endif

#endif
