#ifndef SDK4_ZELOS2_TYPES_H_
#define SDK4_ZELOS2_TYPES_H_

#include <SDK4/SDK4PlatformTypes.h>
	
#ifdef __cplusplus
extern "C" {
#endif

	struct ZELOS2_KNEEPOINT
	{
		uint32_t uKp2ToEoiInMikroSeconds;
		uint32_t uKp1ToEoiInMikroSeconds;
	};

	struct ZELOS2_VCLIPPING
	{
		uint32_t uClippingVoltage1;
		uint32_t uClippingVoltage2;
		uint32_t uClippingVoltage3;
	};


#ifdef __cplusplus
}
#endif

#endif
