#ifndef SDK4_FDC10_TYPES_H_
#define SDK4_FDC10_TYPES_H_

#include <SDK4/SDK4PlatformTypes.h>

#ifdef __cplusplus
extern "C" {
#endif


/// <summary>
/// SDK4 datatype for Binning
/// </summary>
	struct SDK4_KBINNING
	{
		uint32_t horizontal;
		uint32_t vertical;
	};




/// <summary>
/// SDK4 datatype for the index of the Device function
/// </summary>
	struct SDK4_KDEVICESELECT
	{
		uint32_t deviceID;
		uint32_t regAddress;
		uint32_t format;
	};

/// <summary>
/// SDK4 datatype for Circular Blanking and Overlay Position
/// </summary>
	struct SDK4_KCIRCULARPOSITION
	{
		int32_t xPos;
		int32_t yPos;
	};

/// <summary>
/// SDK4 datatype for Circular Measure Window Area
/// </summary>
	struct SDK4_KMWCIRCLE
	{
        uint32_t radius;
        uint32_t width;
        SDK4_KCIRCULARPOSITION center;
		uint32_t vStretch;
	};


/// <summary>
/// SDK4 datatype for Defectpixel Selection
/// </summary>
	struct SDK4_KDEFECTPIXELSELECT
	{
		uint32_t pixelNumber;
		uint32_t store;
	};

/// <summary>
/// Hires3XR datatype for Motor Control Ramp Configuration
/// </summary>
	struct SDK4_KRAMPCONFIG
	{
		uint32_t delay;
		uint32_t steps;
	};


	struct SDK4_AFE_SELECT
	{
		uint32_t index;
		uint32_t store;
	};

	struct SDK4_AFE_PARAM
	{
		uint32_t h1Pol;
		uint32_t h1Pos;
		uint32_t h1Neg;
		uint32_t rgPol;
		uint32_t rgPos;
		uint32_t rgNeg;
		uint32_t SHP;
		uint32_t SHD;
		uint32_t DOUTP;
		uint32_t tClkDelay;
	};

	typedef int32_t SDK4_ENUM_DEFECTPIXEL_MODE;

	/// <summary>
	/// SDK4 datatype for Defectpixel Characterisation
	/// </summary>
		struct SDK4_KDEFECTPIXEL
		{
			SDK4_KPOSITION pos;
			SDK4_ENUM_DEFECTPIXEL_MODE mode;
		};

	

	typedef int32_t SDK4_ENUM_LUTMODE;

	typedef int32_t SDK4_ENUM_MEASUREVALUE;

	typedef int32_t SDK4_ENUM_BINNING;

	typedef int32_t SDK4_ENUM_IMAGEHOLD;

	typedef int32_t SDK4_ENUM_RECURSIVEFILTER;

	typedef int32_t SDK4_ENUM_IOMODE;

	typedef int32_t SDK4_ENUM_IOPORT;

	typedef int32_t SDK4_ENUM_IOPOLARITY;

	typedef int32_t SDK4_ENUM_MEASUREWINDOWSELECT;

	// Hires Types
	typedef int32_t SDK4_ENUM_SHADINGSELECT;
	typedef int32_t SDK4_ENUM_ABC;
	typedef int32_t SDK4_ENUM_ABC_SELECT;
	typedef int32_t SDK4_ENUM_TESTPATTERN;
	typedef int32_t SDK4_ENUM_PATTERNGENERATOR_SELECT;
	typedef int32_t SDK4_ENUM_DEFPIXELCOMPENSATION;
	typedef int32_t SDK4_ENUM_MOTORCONTROL;
	typedef int32_t SDK4_ENUM_MOTORSELECT;

#ifdef __cplusplus
}
#endif

#endif
