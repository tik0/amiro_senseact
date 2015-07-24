#ifndef SDK4_APITYPES_H_
#define SDK4_APITYPES_H_

#include <SDK4/SDK4PlatformTypes.h>
	
#ifdef __cplusplus
extern "C" {
#endif

/// <summary> 
/// SDK4 Error Codes 
/// </summary>
/// <remarks>
/// A function, that was called successfully, returns SDK4_ERR_SUCCESS; 
///	in case of an error one of the negative error codes is returned.
///	</remarks>
	enum SDK4_ERROR_ID 
	{
		SDK4_ERR_SUCCESS            = 0,
		SDK4_ERR_ERROR              = -1001,
		SDK4_ERR_NOT_INITIALIZED	= -1002,
		SDK4_ERR_NOT_IMPLEMENTED    = -1003,
		SDK4_ERR_RESOURCE_IN_USE	= -1004,
		SDK4_ERR_ACCESS_DENIED      = -1005,
		SDK4_ERR_INVALID_HANDLE     = -1006,
		SDK4_ERR_INVALID_ID         = -1007,
		SDK4_ERR_NO_DATA            = -1008,
		SDK4_ERR_INVALID_PARAMETER  = -1009,
		SDK4_ERR_IO					= -1010,
		SDK4_ERR_TIMEOUT            = -1011,
		SDK4_ERR_CANTSETFEATURE		= -1012,
		SDK4_ERR_CANTGETFEATURE		= -1013,
		SDK4_ERR_OUTOFRANGE			= -1014,
		SDK4_ERR_DISABLED			= -1015,
		SDK4_ERR_ACQUISITIONRUNNING = -1016,
		//
		SDK4_ERR_GEV_STATUS_NO_MSG			= -1017,
		SDK4_ERR_GEV_STATUS_MSG_MISMATCH	= -1018,
		SDK4_ERR_GEV_STATUS_TRANSPORT		= -1019,
		//
		SDK4_ERR_INVALID_BUFFER_SIZE = -1201,
		SDK4_ERR_INVALID_ADDRESS     = -1202,
		SDK4_ERR_INVALID_INTERFACE   = -1203
	};
	typedef int32_t SDK4_ERROR;  

/// <summary> 
/// SDK4 Access IDs 
/// </summary>
	enum ENUM_DEVICE_ACCESS_ID
	{		
		DEVICEACCESS_READONLY		=2,         /// Open the device read only. All Port functions can only read from the device.
		DEVICEACCESS_CONTROL		=3,         ///< Open the device in a way that other hosts/processes can have read only access to the device. Device access level is read/write for this process.
		DEVICEACCESS_EXCLUSIVE		=4,         ///< Open the device in a way that only this host/process can have access to the device. Device access level is read/write for this process.
		DEVICEACCESS_CONTROL_HD		=0x80000003,  //HD enable       
		DEVICEACCESS_EXCLUSIVE_HD	=0x80000004          
	};
 	typedef int32_t ENUM_DEVICE_ACCESS;  

/// <summary> 
/// SDK4 Access Status IDs
/// </summary>
	enum ENUM_DEVICE_ACCCESS_STATUS_ID
	{
		DEVICE_ACCESS_STATUS_UNKNOWN   = 0,   ///< The device accessibility is not known
		DEVICE_ACCESS_STATUS_READWRITE = 1,   ///< The device is available for read/write access
		DEVICE_ACCESS_STATUS_READONLY  = 2,   ///< The device is available for readonly access
		DEVICE_ACCESS_STATUS_NOACCESS  = 3,   ///< The device is not accessible
	};
	typedef int32_t ENUM_DEVICE_ACCESS_STATUS;

	enum SDK4_ENUM_DEVICE_INFO_ID
	{
		SDK4_DEVICE_INFO_ID = 0,				/* STRING     Unique ID of the device. */
		SDK4_DEVICE_INFO_VENDOR = 1,			/* STRING     Device vendor name. */
		SDK4_DEVICE_INFO_MODEL = 2,			/* STRING     Device model name. */
		SDK4_DEVICE_INFO_TLTYPE = 3,			/* STRING     Transport layer technology that is supported. */
		SDK4_DEVICE_INFO_DISPLAYNAME = 4,			/* STRING     String containing a display name for the device ( including a unique id ) */
		SDK4_DEVICE_INFO_USER_DEFINED_NAME = 6,		/* STRING     String containing the user defined name  */
		SDK4_DEVICE_INFO_SERIAL_NUMBER = 7,			/* STRING     String containing the device's serial number */
		SDK4_DEVICE_INFO_GEV_IP_ADDRESS = 1000,
		SDK4_DEVICE_INFO_GEV_SUBNET_MASK = 1000 + 1,
		SDK4_DEVICE_INFO_GEV_GATEWAY = 1000 + 2,
		SDK4_DEVICE_INFO_GEV_MACADDRESS = 1000 + 3,
	};
	typedef int32_t SDK4_ENUM_DEVICE_INFO_CMD;

	enum ENUM_CHUNKDATA_ID
	{
		CHUNKDATA_EXPOSURETIME = 0,				/* STRING     Unique ID of the device. */
	};
	typedef int32_t ENUM_CHUNKDATA_CMD;

/// <summary> 
/// SDK4 Media Type IDs
/// </summary>
	enum ENUM_MEDIATYPE_ID
	{
		MEDIATYPE_VIDEOFRAME,
		MEDIATYPE_VIDEOSTREAM,
		MEDIATYPE_AUDIOSTREAM
	};
	typedef int32_t ENUM_MEDIATYPE;

/// <summary> 
/// SDK4 Pixelformat IDs
/// </summary>
    enum ENUM_PIXELFORMAT_ID
    {

/*
//N1:00 Mono
//N1:01 ColorFlag
//N1:02 SignatureFlag
//N2:bpp
//N3:enum#

PIXELFORMAT_RGB888	= 0x01180012,
PIXELFORMAT_BGR888	= 0x01180013,
PIXELFORMAT_RGB8888	= 0x01200014,
PIXELFORMAT_BGR8888	= 0x01200015,
PIXELFORMAT_RGB24	= 0x01180001,
PIXELFORMAT_RGB32	= 0x01200002,
PIXELFORMAT_YUV		= 0x01100003,   // 16 Bit YUV unsigniert
PIXELFORMAT_UYVY	= 0x0110000f,   // 16 Bit YUV unsigniert
PIXELFORMAT_Y8		= 0x00080004,
PIXELFORMAT_Y10		= 0x00100005,   // 10 Bit s/w unsigniert
PIXELFORMAT_Y12		= 0x00100006,   // 12 Bit s/w (nur die unteren 12 Bit sind belegt)
PIXELFORMAT_Y12P	= 0x00100010,   // 12 Bit s/w (nur die unteren 12 Bit sind belegt)
PIXELFORMAT_Y12_MSB = 0x00100012,   // 12 Bit s/w (nur die oberen 12 Bit sind belegt)
PIXELFORMAT_Y14		= 0x00100007,   // 14 Bit s/w unsigniert
PIXELFORMAT_Y14_MSB = 0x00100013,   // 14 Bit s/w -"-
PIXELFORMAT_Y16		= 0x00100008,   // 16 Bit s/w
PIXELFORMAT_D8		= 0x00080014,   // 12 Bit s/w (nur die unteren 12 Bit sind belegt)
PIXELFORMAT_D12		= 0x0010000d,   // 12 Bit s/w (nur die unteren 12 Bit sind belegt)
PIXELFORMAT_D14     = 0x0010000e,    // 14 Bit s/w (nur die unteren 14 Bit sind belegt)
PIXELFORMAT_D16     = 0x00100011,    // 16 Bit s/w (nur die unteren 14 Bit sind belegt)

*/
	//
	// GigEVision Pixel Formats
	//
		PIXELFORMAT_Y8 = 0x01080001,
		PIXELFORMAT_Y10 = 0x01100003,   // 10 Bit s/w unsigniert
		PIXELFORMAT_Y10P = 0x010C0004,   // 10 Bit s/w unsigniert
		PIXELFORMAT_Y12 = 0x01100005,   // 12 Bit s/w (nur die unteren 12 Bit sind belegt)
		PIXELFORMAT_Y12P = 0x010C0006,   // 12 Bit s/w (nur die unteren 12 Bit sind belegt)
		PIXELFORMAT_Y14 = 0x01100025,   // 14 Bit s/w unsigniert
		PIXELFORMAT_Y16 = 0x01100007,   // 16 Bit s/w

		PIXELFORMAT_RGB888 = 0x02180014,	
		PIXELFORMAT_BGR888	= 0x02180015,
		PIXELFORMAT_RGB8888 = 0x02200016,
		PIXELFORMAT_BGR8888 = 0x02200017,
		PIXELFORMAT_YUV = 0x02100032,   // YUV422_YUYV       
		PIXELFORMAT_UYVY = 0x0210001F,   // YUV422_UYVY      

	//
	// Custom Pixel Formats 
	//


	PIXELFORMAT_Y12_MSB = 0x81100005,   // 12 Bit s/w (nur die oberen 12 Bit sind belegt)
	PIXELFORMAT_Y14_MSB = 0x81100025,   // 14 Bit s/w -"-

	PIXELFORMAT_Y12S = 0xC10C0006,   // 12 Bit s/w signiert (nur die unteren 12 Bit sind belegt)
	PIXELFORMAT_Y14S = 0xC1100025,   // 14 Bit s/w signiert
	PIXELFORMAT_Y16S = 0xC1100007,   // 16 Bit s/w signiert
	PIXELFORMAT_YUVS = 0xC2100032,   // 16 Bit YUV signiert                

	PIXELFORMAT_YUV420 = 0x820C0001,   // YUV420       
	PIXELFORMAT_RGB24 = 0x82180014,
	PIXELFORMAT_RGB32 = 0x82200016,

	PIXELFORMAT_D8 = 0x82080001,   // 12 Bit s/w (nur die unteren 12 Bit sind belegt)
	PIXELFORMAT_D12 = 0x82100005,   // 12 Bit s/w (nur die unteren 12 Bit sind belegt)
	PIXELFORMAT_D14 = 0x82100025,    // 14 Bit s/w (nur die unteren 14 Bit sind belegt)
	PIXELFORMAT_D16 = 0x82100007,    // 16 Bit s/w (nur die unteren 14 Bit sind belegt)

	};
 	typedef int32_t ENUM_PIXELFORMAT;  


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


#ifdef __cplusplus
}
#endif

#endif
