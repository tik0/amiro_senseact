#ifndef SDK4_FDC10_ENUMS_H_
#define SDK4_FDC10_ENUMS_H_

#include <SDK4/SDK4PlatformTypes.h>
	
#ifdef __cplusplus
extern "C" {
#endif


/// <summary>
/// SDK4 enumeration for Readout Arae Domains
/// </summary>
enum SDK4_ENUM_ROADOMAIN_ID
{
	SDK4_ENUM_ROADOMAIN_MIN = 0,
	SDK4_ENUM_ROADOMAIN_MAX = 1,
	SDK4_ENUM_ROADOMAIN_ACTIVE = 2
};

enum SDK4_ENUM_LINESTYLE_ID
{
	SDK4_ENUM_LINESTYLE_SOLID = 0,
	SDK4_ENUM_LINESTYLE_DASHED = 1
};

enum SDK4_ENUM_OVERLAYWIDTH_ID
{
	SDK4_ENUM_OVERLAYWIDTH_PIXEL1 = 0,
	SDK4_ENUM_OVERLAYWIDTH_PIXEL3 = 1,
	SDK4_ENUM_OVERLAYWIDTH_PIXEL5 = 2,
	SDK4_ENUM_OVERLAYWIDTH_PIXEL7 = 3
};


/// <summary> 
/// SDK4 enumeration for Defectpixel Mode
/// </summary>
	enum SDK4_ENUM_DEFECTPIXEL_MODE_ID
	{
		SDK4_ENUM_DEFECTPIXEL_COMPENSATION_OFF	= 0,
		SDK4_ENUM_DEFECTPIXEL_COMPENSATION_ON	= 1,
		SDK4_ENUM_DEFECTPIXEL_HIGHLIGHT			= 2
	};


/// <summary> 
/// SDK4 enumeration for ACC Mode
/// </summary>
	enum SDK4_ENUM_ACC_ID
	{
		SDK4_ENUM_ACC_OFF			= 0,
		SDK4_ENUM_ACC_PUSHDRE		= 1,
		SDK4_ENUM_ACC_AUTODRE		= 2,
		SDK4_ENUM_ACC_HISTOGRAM		= 3,
		SDK4_ENUM_ACC_AUTOHISTOGRAM	= 4
	};

/// <summary> 
/// SDK4 enumeration for AET Mode
/// </summary>
	enum SDK4_ENUM_AET_ID 
	{
		SDK4_ENUM_AET_OFF		= 0,
		SDK4_ENUM_AET_ON		= 1,
		SDK4_ENUM_AET_PUSH		= 2,
		//SDK4_ENUM_AET_ON_EXT	= 3
	};

/// <summary> 
/// SDK4 enumeration for Light Source Type
/// </summary>
	enum SDK4_ENUM_LIGHTSOURCE_ID
	{
		SDK4_ENUM_LIGHTSOURCE_DAYLIGHT	= 0,
		SDK4_ENUM_LIGHTSOURCE_HALOGEN	= 1,
		SDK4_ENUM_LIGHTSOURCE_WHITELED	= 2,
		SDK4_ENUM_LIGHTSOURCE_LINEAR	= 3
	};

/// <summary> 
/// SDK4 enumeration for Whiteset Mode
/// </summary>
	enum SDK4_ENUM_WHITESET_ID
	{
		SDK4_ENUM_WHITESET_OFF	= 0,
		SDK4_ENUM_WHITESET_ON	= 1,
		SDK4_ENUM_WHITESET_PUSH	= 2
	};

/// <summary> 
/// SDK4 enumeration for Exposure Base Values
/// </summary>
	enum SDK4_ENUM_EXPOSUREBASE_ID
	{
		SDK4_ENUM_EXPOSUREBASE_PIXELCLOCK	= 0,
		SDK4_ENUM_EXPOSUREBASE_1us			= 1,
		SDK4_ENUM_EXPOSUREBASE_10us			= 2,
		SDK4_ENUM_EXPOSUREBASE_100us		= 3,
		SDK4_ENUM_EXPOSUREBASE_1ms			= 4,
		SDK4_ENUM_EXPOSUREBASE_10ms			= 5,
		SDK4_ENUM_EXPOSUREBASE_100ms		= 6,
		SDK4_ENUM_EXPOSUREBASE_1s			= 7
	};

/// <summary> 
/// SDK4 enumeration for Gamma Values
/// </summary>
	enum SDK4_ENUM_LUTMODE_ID
	{
		SDK4_ENUM_LUTMODE_LINEAR	= 0,
		SDK4_ENUM_LUTMODE_GAMMA		= 1,
		SDK4_ENUM_LUTMODE_USERTABLE	= 2
	};

/// <summary> 
/// SDK4 enumeration for Measure Values
/// </summary>
	enum SDK4_ENUM_MEASUREVALUE_ID
	{
		SDK4_ENUM_MEASUREVALUE_MIN		= 0,
		SDK4_ENUM_MEASUREVALUE_MEAN		= 1,
		SDK4_ENUM_MEASUREVALUE_MAX		= 2
	};

/// <summary> 
/// SDK4 enumeration for ColorCoding of Color Cameras
/// </summary>
	enum SDK4_ENUM_COLORCODING_C_ID
	{
		SDK4_ENUM_COLORCODING_YUV422	= 51,
		SDK4_ENUM_COLORCODING_UYV422	= 52,
		SDK4_ENUM_COLORCODING_RGB24		= 101,
		SDK4_ENUM_COLORCODING_RGB32		= 102,
		SDK4_ENUM_COLORCODING_RGB888	= 110,
		SDK4_ENUM_COLORCODING_BGR888	= 111,
		SDK4_ENUM_COLORCODING_D8		= 150,
		SDK4_ENUM_COLORCODING_D12		= 151,
		SDK4_ENUM_COLORCODING_D12P		= 152,
		SDK4_ENUM_COLORCODING_D14		= 153,
		SDK4_ENUM_COLORCODING_D16		= 155,
	};

/// <summary> 
/// SDK4 enumeration for ColorCoding of B/W Cameras
/// </summary>
	enum SDK4_ENUM_COLORCODING_BW_ID
	{
		SDK4_ENUM_COLORCODING_Y8	= 0,
		SDK4_ENUM_COLORCODING_Y10	= 2,
		SDK4_ENUM_COLORCODING_Y12	= 4,
		SDK4_ENUM_COLORCODING_Y14	= 6,
		SDK4_ENUM_COLORCODING_Y16	= 8,
		SDK4_ENUM_COLORCODING_Y10P = 12,
		SDK4_ENUM_COLORCODING_Y12P = 14,
		SDK4_ENUM_COLORCODING_Y12_MSB	= 24, 
		SDK4_ENUM_COLORCODING_Y14_MSB	= 26, 
	};

/// <summary> 
/// SDK4 enumeration for Binning Modes
/// </summary>
	enum SDK4_ENUM_BINNING_ID
	{
		SDK4_ENUM_BINNING_1x = 1,
		SDK4_ENUM_BINNING_2x = 2,
		SDK4_ENUM_BINNING_3x = 3,
		SDK4_ENUM_BINNING_4x = 4,
		SDK4_ENUM_BINNING_5x = 5,
		SDK4_ENUM_BINNING_6x = 6,
		SDK4_ENUM_BINNING_7x = 7,
		SDK4_ENUM_BINNING_8x = 8
	};


/// <summary> 
/// SDK4 enumeration for Switches
/// </summary>
	/*
	enum SDK4_ENUM_SWITCH_ID
	{
		SDK4_ENUM_SWITCH_OFF = 0,
		SDK4_ENUM_SWITCH_ON = 1
	};	
	typedef int32_t SDK4_ENUM_SWITCH;
*/
/// <summary> 
/// SDK4 enumeration for Image Hold Values
/// </summary>
	enum SDK4_ENUM_IMAGEHOLD_ID
	{
		SDK4_ENUM_IMAGEHOLD_LIVE = 0,
		SDK4_ENUM_IMAGEHOLD_HOLD = 1
	};
	
/// <summary> 
/// SDK4 enumeration for Recursive Filter Values
/// </summary>
	enum SDK4_ENUM_RECURSIVEFILTER_ID
	{
		SDK4_ENUM_RECURSIVEFILTER_FACTOR1 = 0,
		SDK4_ENUM_RECURSIVEFILTER_FACTOR2 = 1,
		SDK4_ENUM_RECURSIVEFILTER_FACTOR4 = 2,
		SDK4_ENUM_RECURSIVEFILTER_FACTOR8 = 3,
		SDK4_ENUM_RECURSIVEFILTER_FACTOR16 = 4
	};

/// <summary> 
/// SDK4 enumeration for Overlay Selection
/// </summary>
	enum SDK4_ENUM_OVERLAYSELECT_ID
	{
		SDK4_ENUM_OVERLAYSELECT_LINEGENERATOR1_ALLLINES = 0,
		SDK4_ENUM_OVERLAYSELECT_LINEGENERATOR2			= 1,
		SDK4_ENUM_OVERLAYSELECT_CIRCLEGENERATOR1		= 2,
		SDK4_ENUM_OVERLAYSELECT_CIRCLEGENERATOR2		= 3
	};
	//typedef int32_t SDK4_ENUM_OVERLAYSELECT;

/// <summary> 
/// SDK4 enumeration for Test Patterns of Color Cameras
/// </summary>
	enum SDK4_ENUM_PATTERN_C_ID
	{
		SDK4_ENUM_PATTERN_C_NORMAL = 0,
		SDK4_ENUM_PATTERN_C_COLORBARS = 1,
	};


/// <summary> 
/// SDK4 enumeration for Test Patterns of B/W Cameras
/// </summary>
	enum SDK4_ENUM_PATTERN_BW_ID
	{
		SDK4_ENUM_PATTERN_BW_NORMAL = 0,
		SDK4_ENUM_PATTERN_BW_OFF = 0,
		SDK4_ENUM_PATTERN_BW_BARS = 1,
		SDK4_ENUM_PATTERN_BW_BURST = 2,
		SDK4_ENUM_PATTERN_BW_BARSBURST = 3
	};

/// <summary> 
/// SDK4 enumeration for I/O Modes
/// </summary>
	enum SDK4_ENUM_IOMODE_ID
	{
		SDK4_ENUM_IOMODE_INPUT = 0,
		SDK4_ENUM_IOMODE_OUTPUT = 1,
		SDK4_ENUM_IOMODE_NA = 255
	};

/// <summary> 
/// SDK4 enumeration for I/O Port Modes
/// </summary>
	enum SDK4_ENUM_IOPORT_ID
	{
		SDK4_ENUM_IOPORT_LOWLEVEL  = 0,
		SDK4_ENUM_IOPORT_HIGHLEVEL = 1,
		SDK4_ENUM_IOPORT_TRIGGERIN = 2,
		SDK4_ENUM_IOPORT_READYOUT  = 3,
		SDK4_ENUM_IOPORT_EXPOSEOUT = 4
		//SDK4_ENUM_IOPORT_EXTSYNCIN = 5,
		//SDK4_ENUM_IOPORT_EXTSYNCLOCKED = 6
	};

/// <summary> 
/// SDK4 enumeration for I/O Polarity
/// </summary>
	enum SDK4_ENUM_IOPOLARITY_ID
	{
		SDK4_ENUM_IOPOLARITY_POSITIVE = 0,
		SDK4_ENUM_IOPOLARITY_NEGATIVE = 1
	};


/*	
/// <summary> 
/// Unsupported
/// SDK4 enumeration for IP Configuration Modes
/// </summary>
	enum SDK4_FLAGS_IPCONFIG_ID
	{
		SDK4_FLAGS_IPCONFIG_STATIC	= 1,
		SDK4_FLAGS_IPCONFIG_DHCP	= 2,
		SDK4_FLAGS_IPCONFIG_LLA		= 4
	};
	typedef int32_t SDK4_FLAGS_IPCONFIG;
*/


/// <summary> 
/// SDK4 enumeration for Measure Windows
/// </summary>
	enum SDK4_ENUM_MEASUREWINDOWSELECT_ID
	{
		SDK4_ENUM_MEASUREWINDOW1		= 0,
		SDK4_ENUM_MEASUREWINDOW2		= 1
	};

#ifdef __cplusplus
}
#endif

#endif
