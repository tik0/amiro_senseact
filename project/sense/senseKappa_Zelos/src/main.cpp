//acquire.cpp

#include <stdio.h>
#include <cstdio>
#include <cstring>

#include <SDK4/SDK4AcquireApi.h>
#include <SDK4/SDK4ZelosApi.h>
#include "SDK4SaveBuffer.h"

#include <vector>
#include <iostream>
#include <string.h>
#include <algorithm>

// Boost
#include <boost/program_options.hpp>

using namespace std;

#define _CHECK(f) {SDK4_ERROR err=(f);if(SDK4_ERR_SUCCESS!=err){cerr<< #f <<" failed: "<<err<<endl;(SDK4CloseLib());}}
#define NBUFFERS 10
//#define NCOUNT 100
int NCOUNT = 5;

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

const char* EnumExposureModeToString(SDK4_ENUM_EXPOSUREMODE iExpMode)
{
	switch (iExpMode)
	{
	case ZELOS_ENUM_EXPOSUREMODE_FREERUNNINGPARALLEL:
		return "ZELOS_ENUM_EXPOSUREMODE_FREERUNNINGPARALLEL";
	case ZELOS_ENUM_EXPOSUREMODE_FREERUNNINGSEQUENTIAL:
		return "ZELOS_ENUM_EXPOSUREMODE_FREERUNNINGSEQUENTIAL";
	case ZELOS_ENUM_EXPOSUREMODE_RESETRESTART:
		return "ZELOS_ENUM_EXPOSUREMODE_RESETRESTART";
	case ZELOS_ENUM_EXPOSUREMODE_FRAMEONDEMAND:
		return "ZELOS_ENUM_EXPOSUREMODE_FRAMEONDEMAND";
	default:
		return "ZELOS_ENUM_EXPOSUREMODE_UNKNOWN";
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

const char* EnumSettingsToString(SDK4_ENUM_SETTINGS v)
{
	switch (v)
	{
	case SDK4_ENUM_SETTINGS_FACTORYSETTINGS: return "Factory"; break;
	case SDK4_ENUM_SETTINGS_USERSETTINGS1: return "User1"; break;
	case SDK4_ENUM_SETTINGS_USERSETTINGS2: return "User2"; break;
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

using namespace cv;

int main(int argc, char* argv[])
{
	std::string prefix("Bilder_Zelos_02150M_CP81030/");
	//std::string zelos2150M_GV("Zelos-02150M::CP81030");
	//std::string zelos2150C_GV("Zelos-02150C::CP81389");
	std::string cameraDevice("Zelos-02150M::CP81030");

	bool   	cfgSaveRawData 	= false; // save as raw or bmp

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
	("d_device,d", po::value < std::string > (&cameraDevice), "device to be opened, default = Zelos-02150M::CP81030")
	("p_path,p", po::value < std::string > (&prefix), "path where the pictures are saved")
	("raw,r", po::value < bool > (&cfgSaveRawData), "save rawData or not, default = false")
	("n_pictureCount,n", po::value < int > (&NCOUNT), "pictures to be captured, default = 30000");

	// allow to give the value as a positional argument
	//po::positional_options_description p;
	//p.add("value", 1);

	po::variables_map vm;
	po::store( po::command_line_parser(argc, argv).options(options).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
	  std::cout << options << "\n";
	  exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

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
	DS_HANDLE hDataStream;
	BUFFER_HANDLE hBuffer[NBUFFERS];
	uint32_t uPayloadSize = 0;
	uint32_t uPacketSize;

	//image parms
	int32_t width = 0;
	int32_t height = 0;
	ENUM_PIXELFORMAT format = 0;

	//Stat Parm
	int nComplete = 0;
	int nIncomplete = 0;
	int nTimedOut = 0;

	//Ctrl Parms
	SDK4_ENUM_SETTINGS enumStartupSetting;
	SDK4_ENUM_EXPOSUREMODE enumExposureMode;
	SDK4_ENUM_AGC agc;
	uint32_t gain;
	SDK4_ENUM_AET aet;
	SDK4_KEXPOSURE kExposure;
	uint32_t uReadoutTime;
	SDK4_ENUM_COLORCODING enumColorCoding;
	uint32_t uPixelClock;

	uint32_t uExposureTime;
	uint32_t uSharpness;
	uint32_t uContrast;
	uint32_t uBrightness;
	int32_t iLUTInterpolatorIndex;
	SDK4_ENUM_LUTMODE enumLUTMode;

	//Buffer Parms
	BUFFER_HANDLE hTempBuffer;
	void* pBuffer;
	uint32_t size;
	int32_t bComplete;
	uint32_t nFrameID;
	char filename[256];
	uint32_t firstFrameID = 0;

	// Configuration Params

	double 	cfgFramerate 	= 30;
	uint32_t 	cfgExposureTime	= 33333;

	cout << "[SDK4 Acquire] " << endl;

	_CHECK(SDK4InitLib());
	_CHECK(SDK4GetNumDevices(&numDevices));

	cout << endl << "Selecting camera device:" << "\n" << "\n";

	for (int32_t i = 0; i < numDevices; i++)
	{
		char sName[256];
		int32_t sizeName = sizeof(sName);
		_CHECK(SDK4GetDeviceID(i, sName, &sizeName));
		cout << "[" << i + 1 << "] " << sName << endl;
		devnames.push_back(sName);
	}
	auto position = std::find(devnames.begin(), devnames.end(), cameraDevice) - devnames.begin();

    if( position < devnames.size() ) {
        std::cout << cameraDevice << " found in connected devices at position " << position << '\n';
    }
    else {
        std::cout << cameraDevice << " not found.\n";
        _CHECK(SDK4CloseLib());
        return 0;
    }
//	cout << endl << "[0 to quit] =>";
//	cin >> choice;
//
//	if (!choice){
//		_CHECK(SDK4CloseLib());
//		return 0;
//	}
    cout << "Opening camera device: " << cameraDevice << "\n";
    choice = position + 1;

	_CHECK(SDK4OpenDevice(devnames[choice - 1].c_str(), DEVICEACCESS_EXCLUSIVE, &hDev));
	_CHECK(SDK4DevGetClassID(hDev, sClassId, &sizeClassId));
	_CHECK(SDK4DevGetModel(hDev, sModel, &sizeModel));
	_CHECK(SDK4DevGetDataStream(hDev, 0, &hDataStream));
	_CHECK(SDK4DevGetControl(hDev,&hCtrl));

	cout << endl;
	cout << "ClassID          = " << sClassId << endl;
	cout << "Model            = " << sModel << endl;

	//Set Framerate
	uReadoutTime = uint32_t(10000.0 / cfgFramerate + 0.5);
	_CHECK(SDK4SetReadoutTime(hCtrl, uReadoutTime));
	_CHECK(SDK4GetReadoutTime(hCtrl, &uReadoutTime));
	cout << "FrameRate        = " << 10000.0 / (double)uReadoutTime  << endl;
	cout << "ReadoutTime      = " << uReadoutTime << endl << endl;;

	_CHECK(SDK4GetStartupSettings(hCtrl, &enumStartupSetting));
	cout << "StartupSettings  = " << EnumSettingsToString(enumStartupSetting) << endl;
	cout << "-------------------------------------------"  << endl;

	//Get ExposureMode
	_CHECK(SDK4GetExposureMode(hCtrl, &enumExposureMode));
	cout << "ExposureMode     = " << EnumExposureModeToString(enumExposureMode) << endl;


	//Get AGC/Gain
	_CHECK(SDK4GetAGC(hCtrl, &agc));
	_CHECK(SDK4GetGain(hCtrl, &gain));
	cout << "AGC              = " << agc  << endl;
	cout << "Gain             = " << gain << endl;

	//Get AET/ExposureTime
	_CHECK(SDK4GetAET(hCtrl, &aet));
	_CHECK(SDK4GetExposure(hCtrl, &kExposure));

	//kExposure.base = SDK4_ENUM_EXPOSUREBASE_1us;
	//kExposure.base = SDK4_ENUM_EXPOSUREBASE_PIXELCLOCK;

	//kExposure.counter = cfgExposureTime;
	_CHECK(SDK4SetAutoExposureLevel(hCtrl, 51));
	//_CHECK(SDK4SetExposure(hCtrl, kExposure));

	_CHECK(SDK4GetPixelClock(hCtrl, &uPixelClock));
	uExposureTime = ConvertExposureToMicroSec(kExposure, uPixelClock);
	cout << "AET              = " << aet  << endl;
	cout << "Exposuretime     = " << uExposureTime << " us" << endl;

	//Get ColorCoding
	_CHECK(SDK4GetColorCoding(hCtrl, &enumColorCoding));
	cout << "Colorcoding      = " << EnumColorCodingToString(enumColorCoding) << endl;

	_CHECK(SDK4GetSharpness(hCtrl, &uSharpness));
	_CHECK(SDK4GetContrast(hCtrl, &uContrast));
	_CHECK(SDK4GetBrightness(hCtrl, &uBrightness));
	_CHECK(SDK4GetLUTMode(hCtrl, &enumLUTMode));
	_CHECK(SDK4GetLUTInterpolatorIndex(hCtrl, &iLUTInterpolatorIndex));
	cout << "Sharpness        = " << uSharpness << endl;
	cout << "Contrast         = " << uContrast << endl;
	cout << "Brightness       = " << uBrightness << endl;
	cout << "LUTMode          = " << EnumLUTModeToString(enumLUTMode) << endl;
	cout << "LUTIndex         = " << iLUTInterpolatorIndex << endl;


	_CHECK(SDK4DSGetWidth(hDataStream, &width));
	_CHECK(SDK4DSGetHeight(hDataStream, &height));
	_CHECK(SDK4DSGetPixelFormat(hDataStream, &format));
	_CHECK(SDK4DSGetPayloadSize(hDataStream, &uPayloadSize));
	_CHECK(SDK4GetPacketSize(hCtrl,&uPacketSize));


	cout << endl;
	cout << "Width            = " << dec << width << endl;
	cout << "Height           = " << dec << height << endl;
	cout << "Format           = 0x" << hex << format << endl;
	cout << "PacketSize       = " << dec << uPacketSize << ((uPacketSize < 4000)?"   WARNING: Payload/MTU Size too small !":"") << endl;
	cout << "PayloadSize      = " << uPayloadSize << endl << endl;

	uint32_t gigEVersion = 0;
	char ipaddress[256];
	char subnet[256];
	int32_t sizeIP = sizeof(ipaddress);
	int32_t sizeSub = sizeof(subnet);
	uint32_t port = 0;
	SDK4_ENUM_IOPORT ioPort;
	_CHECK(SDK4GetGigEVersion(hCtrl, &gigEVersion));
	_CHECK(SDK4GetCurrentIPAddress(hCtrl, ipaddress, &sizeIP));
	_CHECK(SDK4GetCurrentSubnetmask(hCtrl, subnet, &sizeSub));

	_CHECK(SDK4GetIoPort(hCtrl, port, &ioPort));
	std::cout << "gigEVersion: " << gigEVersion << "\nipaddress: " << ipaddress << "\nsubnet: " << subnet<< "\n";
	std::cout << "ioPort: " << port << "  "  << (int) ioPort << "\n";
	// Run Acquisition Task

	cout << "Save RawData = " << (cfgSaveRawData? "True": "False")  << endl  << endl;;

	for (int i = 0; i<NBUFFERS; i++)
	{
		_CHECK(SDK4DSAllocAndAnnounceBuffer(hDataStream, uPayloadSize, NULL, &hBuffer[i]));
		_CHECK(SDK4DSQueueBuffer(hDataStream, hBuffer[i]));
	}
	_CHECK(SDK4DevStartAcquisition(hDev));

	for (int n = 0; n < NCOUNT; n++)
	{

		err = SDK4DSWaitForBuffer(hDataStream, &hTempBuffer, 1000);
		switch (err)
		{
		case SDK4_ERR_SUCCESS:
			_CHECK(SDK4BufferGetPtr(hTempBuffer, &pBuffer));
			_CHECK(SDK4BufferGetSize(hTempBuffer, &size));
			_CHECK(SDK4BufferIsComplete(hTempBuffer, &bComplete));
			_CHECK(SDK4BufferGetFrameID(hTempBuffer,&nFrameID));
			if (firstFrameID == 0)
				firstFrameID = nFrameID;

			if (bComplete)
			{
				if (nFrameID%10==0)
					std::cout << "FrameID  =" << nFrameID << std::endl;

				if (cfgSaveRawData)
				{
					_CHECK(SDK4CreateDateTimeFilename(const_cast<char*>(prefix.c_str()),nFrameID,filename,(char*)"raw",256));
					_CHECK(SDK4SaveBufferRaw(pBuffer,size,filename));
				}
				else
				{
					_CHECK(SDK4CreateDateTimeFilename(const_cast<char*>(prefix.c_str()), nFrameID, filename, (char*)"bmp", 256));
					_CHECK(SDK4SaveBuffer(pBuffer, width, height, format, filename));
				}
				nComplete++;
			}
			else
			{
				std::cout << "FrameID  =" << nFrameID << " incomplete" << std::endl;
				nIncomplete++;
			}
			//Queue Buffer again
			_CHECK(SDK4DSQueueBuffer(hDataStream, hTempBuffer));
			break;
		case SDK4_ERR_TIMEOUT:
			nTimedOut++;
			break;
		default:
			break;
		}
	}
	_CHECK(SDK4DevStopAcquisition(hDev));

	std::cout << "FrameStat: Complete=" << nComplete << " Incomplete=" << nIncomplete << " TimedOut=" << nTimedOut << "\nUnderrun (#InputQueueEmpty) = " << nFrameID - firstFrameID + 1 - NCOUNT << std::endl;

	for (int i = 0; i<NBUFFERS; i++)
	{
		_CHECK(SDK4DSRevokeBuffer(hDataStream, hBuffer[i], NULL, NULL));
	}
	_CHECK(SDK4CloseDevice(hDev));

//Exit:
	_CHECK(SDK4CloseLib());
	return 0;

}


