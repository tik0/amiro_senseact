#ifndef SDK4_CONTROLSTRUCTS_H_
#define SDK4_CONTROLSTRUCTS_H_

#include <SDK4/SDK4PlatformTypes.h>
	
#ifdef __cplusplus
extern "C" {
#endif

	//-------------------------------------------------------------------
	//Enums
	//
	/// <summary>
	/// SDK4 enumeration for Control Access Modes
	/// </summary>
	enum SDK4_ENUM_ACCESSMODE_ID
	{
		SDK4_ENUM_ACCESS_USERMODE = 0,
		SDK4_ENUM_ACCESS_MAINTANANCEMODE = 1,
		SDK4_ENUM_ACCESS_ADJUSTMODE = 2
	};
	typedef int32_t SDK4_ENUM_ACCESSMODE;

	/// <summary>
	/// SDK4 enumeration for Operation Modes
	/// </summary>
	enum SDK4_ENUM_OPERATIONMODE_ID
	{
		SDK4_ENUM_OPERATION_BOOT = 0,
		SDK4_ENUM_OPERATION_OPERATION = 1,
		SDK4_ENUM_OPERATION_FAILURE = 2
	};
	typedef int32_t SDK4_ENUM_OPERATIONMODE;

	/// <summary>
	/// SDK4 enumeration for Startup Settings
	/// </summary>
	enum SDK4_ENUM_SETTINGS_ID
	{
		SDK4_ENUM_SETTINGS_FACTORYSETTINGS = 0,
		SDK4_ENUM_SETTINGS_USERSETTINGS1 = 1,
		SDK4_ENUM_SETTINGS_USERSETTINGS2 = 2
	};
	typedef int32_t SDK4_ENUM_SETTINGS;

	/// <summary>
	/// SDK4 enumeration for Switches
	/// </summary>
	enum SDK4_ENUM_SWITCH_ID
	{
		SDK4_ENUM_SWITCH_OFF = 0,
		SDK4_ENUM_SWITCH_ON = 1
	};
	typedef int32_t SDK4_ENUM_SWITCH;


	typedef int32_t SDK4_ENUM_ROADOMAIN;
	typedef int32_t SDK4_ENUM_LINESTYLE;		//solid dash
	typedef int32_t SDK4_ENUM_OVERLAYWIDTH;		// 1,3,5,7 Pixel
	typedef int32_t SDK4_ENUM_AGC;
	typedef int32_t SDK4_ENUM_AET;
	typedef int32_t SDK4_ENUM_ACC;
	typedef int32_t SDK4_ENUM_EXPOSUREMODE;
	typedef int32_t SDK4_ENUM_COLORCODING;
	typedef int32_t SDK4_ENUM_OVERLAYSELECT;	//LG_1,LG_2
	typedef int32_t SDK4_ENUM_OVERLAYCOLOR;		// new Grey15,..
	typedef int32_t SDK4_ENUM_PATTERN;
	typedef int32_t SDK4_ENUM_MFCOLOR; 
	typedef int32_t SDK4_ENUM_LIGHTSOURCE;
	typedef int32_t SDK4_ENUM_WHITESET;
	typedef int32_t SDK4_ENUM_HISTOEQUAL;		//DigitalEnh
	typedef int32_t SDK4_ENUM_EXPOSUREBASE;

	//-------------------------------------------------------------------
	//Types
	//

	/// <summary> 
	/// SDK4 datatype for Control Access Mode
	/// </summary>
	/// <seealso cref="SDK4_ENUM_ACCESSMODE"/>
	struct SDK4_KACCESSMODE
	{
		uint32_t mode;
		uint32_t unlockCode;
	};

	/// <summary> 
	/// SDK4 datatype for Position
	/// </summary>
	struct SDK4_KPOSITION
	{
		uint32_t xPos;
		uint32_t yPos;
	};

	/// <summary> 
	/// SDK4 datatype for Area
	/// </summary>
	struct SDK4_KAREA
	{
		SDK4_KPOSITION start;
		SDK4_KPOSITION end;
	};

/// <summary> 
/// SDK4 datatype for Overlay Size
/// </summary>
	struct SDK4_KOVERLAYSIZE
	{
		SDK4_ENUM_OVERLAYWIDTH width;
        uint32_t length;
	};

/// <summary> 
/// SDK4 datatype for Overlay Style
/// </summary>
	struct SDK4_KSTYLE
	{
		SDK4_ENUM_OVERLAYCOLOR color;
		SDK4_ENUM_LINESTYLE style;
	};

/// <summary> 
/// SDK4 datatype for Image Size
/// </summary>
	struct SDK4_KSIZE
	{
		uint32_t width;
		uint32_t height;
	};


	
/// <summary> 
/// SDK4 datatype for Color Balance
/// </summary>
	struct SDK4_KRGB
	{
		uint32_t red;
		uint32_t green;
		uint32_t blue;
	};

/// <summary> 
/// SDK4 datatype for Version
/// </summary>
	struct SDK4_KVERSION
	{
		uint32_t major;
        uint32_t minor;
	};

/// <summary> 
/// SDK4 datatype for Revision
/// </summary>
	struct SDK4_KREVISION
	{
		uint32_t hardware;
		uint32_t software;
	};

	/// <summary>
	/// SDK4 datatype for Exposure Time
	/// </summary>
	struct SDK4_KEXPOSURE
	{
		SDK4_ENUM_EXPOSUREBASE base;
		uint32_t counter;
	};

#ifdef __cplusplus
}
#endif

#endif
