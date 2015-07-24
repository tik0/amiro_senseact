#ifndef SDK4_ZELOSTYPES_H_
#define SDK4_ZELOSTYPES_H_

#include <SDK4/SDK4PlatformTypes.h>
	
#ifdef __cplusplus
extern "C" {
#endif

/// <summary> 
/// SDK4 enumeration for Exposure Modes
/// </summary>
	enum ZELOS_ENUM_EXPOSUREMODE_ID
	{
		ZELOS_ENUM_EXPOSUREMODE_FREERUNNINGPARALLEL		= 1,
		ZELOS_ENUM_EXPOSUREMODE_FREERUNNINGSEQUENTIAL	= 2,
		ZELOS_ENUM_EXPOSUREMODE_RESETRESTART			= 3,
		ZELOS_ENUM_EXPOSUREMODE_FRAMEONDEMAND			= 4,
		ZELOS_ENUM_EXPOSUREMODE_EXTERNALSYNC			= 5
	};

/// <summary> 
/// SDK4 enumeration for Measure Frame Colors (Monochrome)
/// </summary>
	enum ZELOS_ENUM_MFCOLOR_BW_ID
	{
		ZELOS_ENUM_COLOR_GREY0	= 0,
		ZELOS_ENUM_COLOR_GREY15 = 1,
		ZELOS_ENUM_COLOR_GREY30 = 2,
		ZELOS_ENUM_COLOR_GREY45 = 3,
		ZELOS_ENUM_COLOR_GREY60 = 4,
		ZELOS_ENUM_COLOR_GREY75 = 5,
		ZELOS_ENUM_COLOR_GREY90 = 6,
		ZELOS_ENUM_COLOR_GREY100= 7
	};
/// <summary> 
/// SDK4 enumeration for Measure Frame Colors (Color)
/// </summary>
	enum SDK4_ENUM_MFCOLOR_C_ID
	{
		ZELOS_ENUM_MFCOLOR_C_BLACK		= 0,
		ZELOS_ENUM_MFCOLOR_C_BLUE		= 1,
		ZELOS_ENUM_MFCOLOR_C_RED		= 2,
		ZELOS_ENUM_MFCOLOR_C_MAGENTA	= 3,
		ZELOS_ENUM_MFCOLOR_C_GREEN		= 4,
		ZELOS_ENUM_MFCOLOR_C_CYAN		= 5,
		ZELOS_ENUM_MFCOLOR_C_YELLOW		= 6,
		ZELOS_ENUM_MFCOLOR_C_WHITE		= 7
	};

/// <summary> 
/// SDK4 enumeration for AGC Modes
/// </summary>
	enum ZELOS_ENUM_AGC_ID
	{
		ZELOS_ENUM_AGC_OFF	= 0,
		ZELOS_ENUM_AGC_ON	= 1,
		ZELOS_ENUM_AGC_PUSH	= 2
	};

	//enum ZELOS_ENUM_ACC_ID
	//{
	//	ZELOS_ENUM_ACC_OFF = 0,
	//	ZELOS_ENUM_ACC_ON = 1,
	//	ZELOS_ENUM_ACC_PUSH = 2
	//};

/// <summary> 
/// SDK4 enumeration for Overlay Selection
/// </summary>
	enum ZELOS_ENUM_OVERLAYSELECT_ID
	{
		ZELOS_ENUM_OVERLAYSELECT_LINEGENERATOR1_ALLLINES = 0,
		ZELOS_ENUM_OVERLAYSELECT_LINRGENERATOR2			= 1,
		ZELOS_ENUM_OVERLAYSELECT_CIRCLEGENERATOR1		= 2,
		ZELOS_ENUM_OVERLAYSELECT_CIRCLEGENERATOR2		= 3
	};


#ifdef __cplusplus
}
#endif

#endif