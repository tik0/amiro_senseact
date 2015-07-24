#ifndef SDK4SAVEBMP_H_
#define SDK4SAVEBMP_H_

#include <SDK4/SDK4AcquireApi.h>
#include <string>

#define _CHECK_GC(f) {SDK4_ERROR error=(f);if(SDK4_ERR_SUCCESS!=err){ return error; }}
extern SDK4_ERROR SDK4SaveBuffer(void* pBuffer, int32_t width, int32_t height, ENUM_PIXELFORMAT pixelformat, const char* filename);
extern SDK4_ERROR SDK4CreateDateTimeFilename(char* prefix, uint32_t frameid,char* filename,char* ext,size_t size);
extern SDK4_ERROR SDK4SaveBufferRaw(void* pBuffer, size_t length, const char* filename);


#endif
