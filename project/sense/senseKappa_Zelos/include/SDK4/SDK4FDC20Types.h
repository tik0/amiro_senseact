#ifndef SDK4_FDC20_TYPES_H_
#define SDK4_FDC20_TYPES_H_

#include <SDK4/SDK4PlatformTypes.h>
	
#ifdef __cplusplus
extern "C" {
#endif

	typedef int32_t FDC20_ENUM_BINNINGHV;
	typedef int32_t FDC20_ENUM_MWSELECT;			//FDC10:SDK4_ENUM_MEASUREWINDOWSELECT
	typedef int32_t FDC20_ENUM_MWRESULT;
	typedef int32_t FDC20_ENUM_IBITSELECT;
	typedef int32_t FDC20_ENUM_CCDCOOLING;
	typedef uint32_t FDC20_ENUM_CCDCOOLING_STATUS;

	enum FDC20_ENUM_COMPONENT_SELECT_ID
	{
		FDC20_ENUM_COMPONENT_SELECT_MC = 0,
		FDC20_ENUM_COMPONENT_SELECT_FPGA = 1
	};
	typedef int32_t FDC20_ENUM_COMPONENT_SELECT;

	enum FDC20_ENUM_TEMPERATURE_SELECT_ID
	{
		FDC20_ENUM_TEMPERATURE_SELECT_PCB = 0
	};
	typedef int32_t FDC20_ENUM_TEMPERATURE_SELECT;


#ifdef __cplusplus
}
#endif

#endif
