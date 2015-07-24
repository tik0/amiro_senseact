#ifndef SDK4_ZELOS2_ENUMS_H_
#define SDK4_ZELOS2_ENUMS_H_

#include <SDK4/SDK4PlatformTypes.h>
	
#ifdef __cplusplus
extern "C" {
#endif

	enum ZELOS2_ENUM_CCDCOOLING_ID
	{
		ZELOS2_ENUM_CCDCOOLING_OFF = 0,
		ZELOS2_ENUM_CCDCOOLING_BOOST = 1,
		ZELOS2_ENUM_CCDCOOLING_CONTROLLED = 2
	};

	enum ZELOS2_ENUM_ROADOMAIN_ID
	{
		ZELOS2_ENUM_ROADOMAIN_MIN = 0,
		ZELOS2_ENUM_ROADOMAIN_MAX = 1,
		ZELOS2_ENUM_ROADOMAIN_ACTIVE = 2
	};

	enum ZELOS2_ENUM_LINESTYLE_ID
	{
		ZELOS2_ENUM_LINESTYLE_SOLID = 0,
		ZELOS2_ENUM_LINESTYLE_DASHED = 1
	};

	enum ZELOS2_ENUM_OVERLAYWIDTH_ID
	{
		ZELOS2_ENUM_OVERLAYWIDTH_PIXEL1 = 0,
		ZELOS2_ENUM_OVERLAYWIDTH_PIXEL3 = 1,
		ZELOS2_ENUM_OVERLAYWIDTH_PIXEL5 = 2,
		ZELOS2_ENUM_OVERLAYWIDTH_PIXEL7 = 3
	};

	enum ZELOS2_ENUM_OVERLAYCOLOR_ID
	{
		ZELOS2_ENUM_OVERLAYCOLOR_GREY0  = 0,
		ZELOS2_ENUM_OVERLAYCOLOR_GREY15 = 1,
		ZELOS2_ENUM_OVERLAYCOLOR_GREY30 = 2,
		ZELOS2_ENUM_OVERLAYCOLOR_GREY45 = 3,
		ZELOS2_ENUM_OVERLAYCOLOR_GREY60 = 4,
		ZELOS2_ENUM_OVERLAYCOLOR_GREY75 = 5,
		ZELOS2_ENUM_OVERLAYCOLOR_GREY90 = 6,
		ZELOS2_ENUM_OVERLAYCOLOR_GREY100 = 7,
		ZELOS2_ENUM_OVERLAYCOLOR_BLACK = 0,
		ZELOS2_ENUM_OVERLAYCOLOR_BLUE = 1,
		ZELOS2_ENUM_OVERLAYCOLOR_RED = 2,
		ZELOS2_ENUM_OVERLAYCOLOR_MAGENTA = 3,
		ZELOS2_ENUM_OVERLAYCOLOR_GREEN = 4,
		ZELOS2_ENUM_OVERLAYCOLOR_CYAN = 5,
		ZELOS2_ENUM_OVERLAYCOLOR_YELLOW = 6,
		ZELOS2_ENUM_OVERLAYCOLOR_WHITE = 7
	};

	
	enum ZELOS2_ENUM_EXPOSUREMODE_ID
	{
		ZELOS2_ENUM_EXPOSUREMODE_FREERUNNINGPARALLEL	= 1,
		ZELOS2_ENUM_EXPOSUREMODE_FREERUNNINGSEQUENTIAL	= 2,
		ZELOS2_ENUM_EXPOSUREMODE_RESETRESTART			= 3,
		ZELOS2_ENUM_EXPOSUREMODE_FRAMEONDEMAND			= 4,
	};

	enum ZELOS2_ENUM_OVERLAYSELECT_ID
	{
		ZELOS2_ENUM_OVERLAYSELECT_LINEGENERATOR1 = 0,
		ZELOS2_ENUM_OVERLAYSELECT_LINEGENERATOR2 = 1
	};

	enum ZELOS2_ENUM_MWSELECT_ID
	{
		ZELOS2_ENUM_MWSELECT_AE_WINDOW1 = 0,
		ZELOS2_ENUM_MWSELECT_AE_WINDOW2 = 1,
		ZELOS2_ENUM_MWSELECT_ACC_WINDOW1 = 2,
		ZELOS2_ENUM_MWSELECT_ACC_WINDOW2 = 3,
		ZELOS2_ENUM_MWSELECT_AWB_WINDOW1 = 4
	};

	enum ZELOS2_ENUM_MFCOLOR_ID
	{
		ZELOS2_ENUM_MFCOLOR_GREY0 = 0,
		ZELOS2_ENUM_MFCOLOR_GREY15 = 1,
		ZELOS2_ENUM_MFCOLOR_GREY30 = 2,
		ZELOS2_ENUM_MFCOLOR_GREY45 = 3,
		ZELOS2_ENUM_MFCOLOR_GREY60 = 4,
		ZELOS2_ENUM_MFCOLOR_GREY75 = 5,
		ZELOS2_ENUM_MFCOLOR_GREY90 = 6,
		ZELOS2_ENUM_MFCOLOR_GREY100 = 7,
		ZELOS2_ENUM_MFCOLOR_BLACK = 0,
		ZELOS2_ENUM_MFCOLOR_BLUE = 1,
		ZELOS2_ENUM_MFCOLOR_RED = 2,
		ZELOS2_ENUM_MFCOLOR_MAGENTA = 3,
		ZELOS2_ENUM_MFCOLOR_GREEN = 4,
		ZELOS2_ENUM_MFCOLOR_CYAN = 5,
		ZELOS2_ENUM_MFCOLOR_YELLOW = 6,
		ZELOS2_ENUM_MFCOLOR_WHITE = 7
	};
	enum ZELOS2_ENUM_MWRESULT_ID
	{
		ZELOS2_ENUM_MWRESULT_MIN = 0,
		ZELOS2_ENUM_MWRESULT_MEAN = 1,
		ZELOS2_ENUM_MWRESULT_MAX = 2,
	};



	enum ZELOS2_ENUM_IBITSELECT_ID
	{
		//ZELOS2_ENUM_IBITSELECT_15V0_SH = 0,
		ZELOS2_ENUM_IBITSELECT_15V0 = 1,
		ZELOS2_ENUM_IBITSELECT_n9V0 = 2,
		//ZELOS2_ENUM_IBITSELECT_n15V0_SH = 3,
		ZELOS2_ENUM_IBITSELECT_3V3 = 4,
		ZELOS2_ENUM_IBITSELECT_2V5 = 5,
		ZELOS2_ENUM_IBITSELECT_1V8 = 6,
		ZELOS2_ENUM_IBITSELECT_1V2 = 7,
		ZELOS2_ENUM_IBITSELECT_5V0 = 8,
		//ZELOS2_ENUM_IBITSELECT_TEMPERATURE = 9,
		//ZELOS2_ENUM_IBITSELECT_nVD = 10,
		//ZELOS2_ENUM_IBITSELECT_VSUB = 11,
		//ZELOS2_ENUM_IBITSELECT_FPGA = 12,
		ZELOS2_ENUM_IBITSELECT_EEPROM = 13,
		//ZELOS2_ENUM_IBITSELECT_SRAM = 14,
	};

	enum ZELOS2_ENUM_PATTERN_ID
	{
		ZELOS2_ENUM_PATTERN_OFF_COLOR_MODE = 0,
		ZELOS2_ENUM_PATTERN_CONSUMER = 1,
		ZELOS2_ENUM_PATTERN_GRAYSCALE = 2,
		ZELOS2_ENUM_PATTERN_GRAYSCALE_MARGIN_1 = 3,
		ZELOS2_ENUM_PATTERN_GRAYSCALE_MARGIN_2 = 4,
		ZELOS2_ENUM_PATTERN_EQUALIZER = 5,
		ZELOS2_ENUM_PATTERN_TRIANGLE_4 = 6,
		ZELOS2_ENUM_PATTERN_RESOLUTION = 7,
		ZELOS2_ENUM_PATTERN_GRAYRAMP_VER = 8,
		ZELOS2_ENUM_PATTERN_GRAYRAMP_HOR = 9,
		ZELOS2_ENUM_PATTERN_COLORRAMP = 10,
		ZELOS2_ENUM_PATTERN_SDI_BW = 11,
		ZELOS2_ENUM_PATTERN_COLORBAR_RGB_100 = 12,
		ZELOS2_ENUM_PATTERN_COLORBAR_RGB_75 = 13,
		ZELOS2_ENUM_PATTERN_SDI_COLOR = 14,
		ZELOS2_ENUM_PATTERN_OFF_BW_MODE = 15
	};

	enum ZELOS2_ENUM_BINNINGHV_ID
	{
		ZELOS2_ENUM_BINNINGHV_1X1 = 1,
		ZELOS2_ENUM_BINNINGHV_2X2 = 2,
		ZELOS2_ENUM_BINNINGHV_4X4 = 4
	};

	enum ZELOS2_ENUM_LIGHTSOURCE_ID
	{
		ZELOS2_ENUM_LIGHTSOURCE_DAYLIGHT = 0,
		ZELOS2_ENUM_LIGHTSOURCE_HALOGEN = 1,
		ZELOS2_ENUM_LIGHTSOURCE_WHITELED = 2,
		ZELOS2_ENUM_LIGHTSOURCE_LINEAR = 3,
		ZELOS2_ENUM_LIGHTSOURCE_DAYLIGHT_D50 = 4
	};
	
	enum ZELOS2_ENUM_AGC_ID
	{
		ZELOS2_ENUM_AGC_OFF = 0,
		ZELOS2_ENUM_AGC_ON = 1,
	};

	enum ZELOS2_ENUM_AET_ID
	{
		ZELOS2_ENUM_AET_OFF = 0,
		ZELOS2_ENUM_AET_ON = 1,
	};


	enum ZELOS2_ENUM_ACC_ID
	{
		ZELOS2_ENUM_ACC_OFF = 0,
		ZELOS2_ENUM_ACC_ON = 1,
	};

	enum ZELOS2_ENUM_WHITESET_ID
	{
		ZELOS2_ENUM_WHITESET_OFF = 0,
		ZELOS2_ENUM_WHITESET_ON = 1,
	};

	enum ZELOS2_ENUM_COLORCODING_ID
	{
		//color
		ZELOS2_ENUM_COLORCODING_UYV422	= 52,
		ZELOS2_ENUM_COLORCODING_RGB888	= 110,

		//mono
		ZELOS2_ENUM_COLORCODING_Y8		= 0,
		ZELOS2_ENUM_COLORCODING_Y12		= 4,
		ZELOS2_ENUM_COLORCODING_Y12P 	= 14,
		ZELOS2_ENUM_COLORCODING_Y14		= 6,
		ZELOS2_ENUM_COLORCODING_D8		= 150,
		ZELOS2_ENUM_COLORCODING_D12		= 151,

		//color & mono
		ZELOS2_ENUM_COLORCODING_D14		= 153,

	};
	enum ZELOS2_ENUM_CCDCOOLING_STATUS_ID 
	{
		ZELOS2_ENUM_COOLING_STATUS_HEARTBEAT_MISSING = 0x00000001,
		ZELOS2_ENUM_COOLING_STATUS_TEMPERATURE_DEVIATION = 0x00000002,
		ZELOS2_ENUM_COOLING_STATUS_FAN_DEVIATION = 0x00000004,
		ZELOS2_ENUM_COOLING_STATUS_TEMPERATURE_CONTROL = 0x00000008,
		ZELOS2_ENUM_COOLING_STATUS_TEMPERATURE_LOW_LIMIT = 0x00000010,
		ZELOS2_ENUM_COOLING_STATUS_COMMUNICATION_FAILURE = 0x80000000
	};

#ifdef __cplusplus
}
#endif

#endif
