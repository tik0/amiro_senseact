//acquire.cpp

#include <stdio.h>
#include <cstdio>
#include <cstring>

#include <SDK4/SDK4AcquireApi.h>
#include <SDK4/SDK4ZelosApi.h>


#include <vector>
#include <iostream>
#include <string.h>
using namespace std;

#define _CHECK(f) {SDK4_ERROR err=(f);if(SDK4_ERR_SUCCESS!=err){cerr<< #f <<" failed: "<<err<<endl;goto Exit;}}


const char* EnumColorCodingToString(SDK4_ENUM_COLORCODING iColorCoding)
{
	switch (iColorCoding)
	{
	case SDK4_ENUM_COLORCODING_Y8:
		return "Y8";
	case SDK4_ENUM_COLORCODING_Y12P:
		return "Y12P";
	case SDK4_ENUM_COLORCODING_Y14:
		return "Y14";
	case SDK4_ENUM_COLORCODING_D14:
		return "D14";
	case SDK4_ENUM_COLORCODING_UYV422:
		return "UYV422";
	case SDK4_ENUM_COLORCODING_RGB888:
		return "RGB888";
	case SDK4_ENUM_COLORCODING_BGR888:
		return "BGR888";
	default:
		return "Unknown";
	}
}
const char* EnumLUTModeToString(SDK4_ENUM_LUTMODE mode)
{
	switch (mode)
	{
	case SDK4_ENUM_LUTMODE_LINEAR: return "LINEAR"; break;
	case SDK4_ENUM_LUTMODE_GAMMA: return "GAMMA"; break;
	default: return "UNKNOWN"; break;
	}
}

uint32_t ConvertExposureToMicroSec(SDK4_KEXPOSURE kExposure, uint32_t uPixelClock)
{
	switch (kExposure.base)
	{
	case SDK4_ENUM_EXPOSUREBASE_PIXELCLOCK:
		return uint32_t((double)(kExposure.counter) * 1000000.0 / (double)(uPixelClock));
	case SDK4_ENUM_EXPOSUREBASE_1us:
		return kExposure.counter;
	case SDK4_ENUM_EXPOSUREBASE_10us:
		return kExposure.counter * 10;
	case SDK4_ENUM_EXPOSUREBASE_100us:
		return kExposure.counter * 100;
	case SDK4_ENUM_EXPOSUREBASE_1ms:
		return kExposure.counter * 1000;
	case SDK4_ENUM_EXPOSUREBASE_10ms:
		return kExposure.counter * 10000;
	case SDK4_ENUM_EXPOSUREBASE_100ms:
		return kExposure.counter * 100000;
	case SDK4_ENUM_EXPOSUREBASE_1s:
		return kExposure.counter * 1000000;
	default:
		return 0;
	}
}

int main()
{
	SDK4_ERROR err;
	bool bOpen = false;
	int choice;

	static char sModel[256];
	int32_t sizeModel = sizeof(sModel);

	char sClassId[256];
	int32_t sizeClassId = sizeof(sClassId);
	
	char sDeviceName[256];
	int32_t sizeDeviceName = sizeof(sDeviceName);

	vector<string> devnames;
	int32_t numDevices = 0;

	//Module Handle
	DEV_HANDLE hDev;
	CTRL_HANDLE hCtrl;

	//Config
	bool   		cfgSaveRawData 	= true;
	uint32_t 	cfgGain 		= 0;
	uint32_t 	cfgExposureTime	= 33333;	// usec
	uint32_t 	cfgContrast		= 32;		// linear
	uint32_t 	cfgBrightness	= 0;		// no offset

	SDK4_ENUM_LUTMODE 	cfgLUTMode_YUV422 				= SDK4_ENUM_LUTMODE_GAMMA; // ColorCoding: UYV422
	int32_t 			cfgLUTInterpolatorIndex_YUV422 	= 8;	//0.45
	uint32_t 			cfgSharpness_YUV422 			= 100;

	SDK4_ENUM_LUTMODE 	cfgLUTMode_MONO 				= SDK4_ENUM_LUTMODE_LINEAR; // ColorCoding: Y8,Y14,D14
	int32_t 			cfgLUTInterpolatorIndex_MONO 	= 0;	//1.00
	uint32_t 			cfgSharpness_MONO 				= 0;

	//Ctrl Parms
	SDK4_KEXPOSURE kExposure;
	uint32_t uReadoutTime;
	SDK4_ENUM_COLORCODING enumColorCoding;
	uint32_t uPixelClock;
	uint32_t uExposureTime;

	cout << "[SDK4 Acquire] " << endl;

	_CHECK(SDK4InitLib());
	_CHECK(SDK4GetNumDevices(&numDevices));

	cout << endl << "Select camera device:" << endl << endl;

	for (int32_t i = 0; i < numDevices; i++)
	{
		char sName[256];
		int32_t sizeName = sizeof(sName);
		_CHECK(SDK4GetDeviceID(i, sName, &sizeName));
		cout << "[" << i + 1 << "] " << sName << endl;
		devnames.push_back(sName);
	}
	cout << endl << "[0 to quit] =>";
	cin >> choice; if (!choice) goto Exit;

	_CHECK(SDK4OpenDevice(devnames[choice - 1].c_str(), DEVICEACCESS_EXCLUSIVE, &hDev));
	_CHECK(SDK4DevGetClassID(hDev, sClassId, &sizeClassId));
	_CHECK(SDK4DevGetModel(hDev, sModel, &sizeModel));	
	_CHECK(SDK4DevGetControl(hDev,&hCtrl));

	cout << endl;
	cout << "ClassID          = " << sClassId << endl;
	cout << "Model            = " << sModel << endl;
	
	_CHECK(SDK4LoadSettings(hCtrl, SDK4_ENUM_SETTINGS_FACTORYSETTINGS));				//Load Factory Settings
	_CHECK(SDK4SetExposureMode(hCtrl, ZELOS_ENUM_EXPOSUREMODE_FREERUNNINGPARALLEL));	// Fastest mode upto 30 fps ; max exposuretime is limited to 1/fps
		//Set Gain
	_CHECK(SDK4SetAGC(hCtrl, SDK4_ENUM_SWITCH_OFF));		//agc off
	_CHECK(SDK4SetGain(hCtrl, cfgGain));

	//Set ExposureTime
	_CHECK(SDK4SetAET(hCtrl, SDK4_ENUM_SWITCH_OFF));		//aet off
	_CHECK(SDK4GetPixelClock(hCtrl, &uPixelClock));		
	kExposure.base = SDK4_ENUM_EXPOSUREBASE_1us;
	kExposure.counter = cfgExposureTime;
	_CHECK(SDK4SetExposure(hCtrl, kExposure));
	_CHECK(SDK4GetExposure(hCtrl, &kExposure));
	uExposureTime = ConvertExposureToMicroSec(kExposure, uPixelClock);
	cout << "Exposuretime     = " << uExposureTime << " us" << endl;

	//Set Image Processing Params
	_CHECK(SDK4SetContrast(hCtrl, cfgContrast));
	_CHECK(SDK4SetBrightness(hCtrl,cfgBrightness));
	cout << "Contrast         = " << cfgContrast << endl;
	cout << "Brightness       = " << cfgBrightness << endl;

	//Set ColorCoding
	if (string(sModel).find("C") != std::string::npos)//ColorCam
	{ 
		if (cfgSaveRawData)
		{
			_CHECK(SDK4SetColorCoding(hCtrl, SDK4_ENUM_COLORCODING_D14));		// 14 bit Rawdata - Bayer-Pattern - No ImageProcessing
		}
		else
		{
			_CHECK(SDK4SetColorCoding(hCtrl, SDK4_ENUM_COLORCODING_UYV422));
		}
	}
	else // MonoCam
	{
		if (cfgSaveRawData)
		{
			_CHECK(SDK4SetColorCoding(hCtrl, SDK4_ENUM_COLORCODING_Y14));		// D14 not implemented , use Y14
		}
		else
		{
			_CHECK(SDK4SetColorCoding(hCtrl, SDK4_ENUM_COLORCODING_Y8));
		}
	}
	_CHECK(SDK4GetColorCoding(hCtrl, &enumColorCoding));
	cout << "Colorcoding      = " << EnumColorCodingToString(enumColorCoding) << endl;


	if(enumColorCoding==SDK4_ENUM_COLORCODING_UYV422)
	{
		_CHECK(SDK4SetSharpness(hCtrl, cfgSharpness_YUV422));
		_CHECK(SDK4SetLUTMode(hCtrl, cfgLUTMode_YUV422));
		_CHECK(SDK4SetLUTInterpolatorIndex(hCtrl, cfgLUTInterpolatorIndex_YUV422));
	cout << "Sharpness        = " << cfgSharpness_YUV422 << endl;
		cout << "LUTMode          = " << EnumLUTModeToString(cfgLUTMode_YUV422) << endl;
		cout << "LUTIndex         = " << cfgLUTInterpolatorIndex_YUV422 << endl;
	}
	else
	{
		_CHECK(SDK4SetSharpness(hCtrl, cfgSharpness_MONO));
		_CHECK(SDK4SetLUTMode(hCtrl, cfgLUTMode_MONO));
		cout << "Sharpness        = " << cfgSharpness_MONO << endl;
		cout << "LUTMode          = " << EnumLUTModeToString(cfgLUTMode_MONO) << endl;
		cout << "LUTIndex         = " << cfgLUTInterpolatorIndex_MONO << endl;
	}


	// Store Params to UserSettings1
	_CHECK(SDK4SaveSettings(hCtrl, SDK4_ENUM_SETTINGS_USERSETTINGS1));				//Save to usersettings1
	_CHECK(SDK4SetStartupSettings(hCtrl, SDK4_ENUM_SETTINGS_USERSETTINGS1));		//Startup with usersettings 1


	_CHECK(SDK4CloseDevice(hDev));

Exit:
	_CHECK(SDK4CloseLib());
	return 0;
}


