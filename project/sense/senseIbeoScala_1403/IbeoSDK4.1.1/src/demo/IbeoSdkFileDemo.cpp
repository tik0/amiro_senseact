//======================================================================
/*! \file IbeoSdkFileDemo.cpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date Jun 1, 2012
 *
 * Demo project for reading IDC files and process the data blocks.
 *///-------------------------------------------------------------------

#include <ibeosdk/lux.hpp>
#include <ibeosdk/ecu.hpp>
#include <ibeosdk/minilux.hpp>
#include <ibeosdk/scala.hpp>

#include <ibeosdk/devices/IdcFile.hpp>

#include <iostream>
#include <cstdlib>

//======================================================================

using namespace ibeosdk;

//======================================================================

const ibeosdk::Version::MajorVersion majorVersion(4);
const ibeosdk::Version::MinorVersion minorVersion(1);
const ibeosdk::Version::Revision revision = ibeosdk::Version::Revision(1);
const ibeosdk::Version::PatchLevel patchLevel;
const ibeosdk::Version::Build build;
const std::string info = "IbeoSdkFileDemo";

ibeosdk::Version appVersion(majorVersion, minorVersion, revision, patchLevel, build, info);

IbeoSDK ibeoSDK;

//======================================================================

void file_demo(const std::string& filename);

//======================================================================

TimeConversion tc;

//======================================================================

class AllListener : public ibeosdk::DataListener<FrameEndSeparator>,
                    public ibeosdk::DataListener<ScanLux>,
                    public ibeosdk::DataListener<ScanEcu>,
                    public ibeosdk::DataListener<Scan2208>,
                    public ibeosdk::DataListener<ObjectListLux>,
                    public ibeosdk::DataListener<ObjectListEcu>,
                    public ibeosdk::DataListener<ObjectListScala>,
                    public ibeosdk::DataListener<ObjectListScala2271>,
                    public ibeosdk::DataListener<ObjectListEcuEt>,
                    public ibeosdk::DataListener<Image>,
                    public ibeosdk::DataListener<PositionWgs84_2604>,
                    public ibeosdk::DataListener<MeasurementList2821>,
                    public ibeosdk::DataListener<VehicleStateBasicLux>,
                    public ibeosdk::DataListener<VehicleStateBasicEcu2806>,
                    public ibeosdk::DataListener<VehicleStateBasicEcu>,
                    public ibeosdk::DataListener<DeviceStatus>,
                    public ibeosdk::DataListener<DeviceStatus6303>,
                    public ibeosdk::DataListener<LogMessageError>,
                    public ibeosdk::DataListener<LogMessageWarning>,
                    public ibeosdk::DataListener<LogMessageNote>,
                    public ibeosdk::DataListener<LogMessageDebug> {
public:
	virtual ~AllListener() {}

public:
	//========================================
	void onData(const FrameEndSeparator* const fes)
	{
		logInfo << std::setw(5) << fes->getSerializedSize() << " Bytes  "
				<< "Frame received: # " << fes->getFrameId()
				<< "  Frame time: " << tc.toString(fes->getHeaderNtpTime().toPtime())
				<< std::endl;
	}

	//========================================
	void onData(const ScanLux* const scan)
	{
		logInfo << std::setw(5) << scan->getSerializedSize() << " Bytes  "
				<< "ScanLux received: # " << scan->getScanNumber()
				<< "  ScanStart: " << tc.toString(scan->getStartTimestamp().toPtime())
				<< std::endl;
	}

	//========================================
	void onData(const ScanEcu* const scan)
	{
		logInfo << std::setw(5) << scan->getSerializedSize() << " Bytes  "
				<< "ScanEcu received: # " << scan->getScanNumber()
				<< "  #Pts: " << scan->getNumberOfScanPoints()
				<< "  ScanStart: " << tc.toString(scan->getStartTimestamp().toPtime(), 3)
				<< std::endl;
	}

	//========================================
	void onData(const Scan2208* const scan)
	{
		logInfo << std::setw(5) << scan->getSerializedSize() << " Bytes  "
				<< "Scan2208 received: # " << scan->getScanNumber()
				<< "  #Pts: " << scan->getSubScans().at(0).getNbOfPoints()
				<< "  ScanStart: " << tc.toString(scan->getSubScans().at(0).getStartScanTimestamp().toPtime(), 3)
				<< std::endl;
	}

	//========================================
	void onData(const ObjectListLux* const objList)
	{
		logInfo << std::setw(5) << objList->getSerializedSize() << " Bytes  " << "ObjectListLux received: # " << objList->getNumberOfObjects() << std::endl;
	}

	//========================================
	void onData(const ObjectListEcu* const objList)
	{
		logInfo << std::setw(5) << objList->getSerializedSize() << " Bytes  " << "ObjectListEcu received: # " << objList->getNumberOfObjects() << std::endl;
	}

	//========================================
	void onData(const ObjectListScala* const objList)
	{
		logInfo << std::setw(5) << objList->getSerializedSize() << " Bytes  " << "ObjectListScala received: # " << objList->getNumberOfObjects() << std::endl;
	}

	//========================================
	void onData(const ObjectListScala2271* const objs)
	{
		logInfo << std::setw(5) << objs->getSerializedSize() << " Bytes  "
				<< "ObjectList 2271 received. Scan: " << objs->getScanNumber()
				<< "  ObjLstId: " << int(objs->getObjectListId())
				<< "  #Obj:" << objs->getNumberOfObjects()
				<< std::endl;
	}

	//========================================
	void onData(const ObjectListEcuEt* const objList)
	{
		logInfo << std::setw(5) << objList->getSerializedSize() << " Bytes  " << "ObjectListEcUEts received: # " << objList->getNbOfObjects() << std::endl;
	}

	//========================================
	void onData(const Image* const image)
	{
		logInfo << std::setw(5) << image->getSerializedSize() << " Bytes  " << "Image received: time: " << tc.toString(image->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	void onData(const PositionWgs84_2604* const wgs84)
	{
		logInfo << std::setw(5) << wgs84->getSerializedSize() << " Bytes  "
				<< "PositionWGS84 received: time: " << tc.toString(wgs84->getPosition().getTimestamp().toPtime())
				<< std::endl;
	}

	//========================================
	void onData(const VehicleStateBasicLux* const vsb)
	{
		logInfo << std::setw(5) << vsb->getSerializedSize() << " Bytes  " << "VSB (LUX) received: time: " << tc.toString(vsb->getTimestamp().toPtime()) << std::endl;
	}

	//========================================
	void onData(const VehicleStateBasicEcu2806* const vsb)
	{
		logInfo << std::setw(5) << vsb->getSerializedSize() << " Bytes  "
				<< "VSB (ECU;old) received: time: " << tc.toString(vsb->getTimestamp().toPtime())
				<< std::endl;
	}

	//========================================
	void onData(const VehicleStateBasicEcu* const vsb)
	{
		logInfo << std::setw(5) << vsb->getSerializedSize() << " Bytes  "
				<< "VSB (ECU) received: time: " << tc.toString(vsb->getTimestamp().toPtime())
				<< std::endl;
	}
	//========================================
	void onData(const MeasurementList2821* const ml)
	{
		logInfo << std::setw(5) << ml->getSerializedSize() << " Bytes  "
				<< "MeasurementList received: time: " << tc.toString(ml->getTimestamp().toPtime())
				<< " LN: '" << ml->getListName() << "' GN: '" << ml->getGroupName() << "'" << "Num: " << ml->getMeasList().getSize()
				<< std::endl;

		typedef std::vector<Measurement> MLVector;

		MLVector::const_iterator itMl = ml->getMeasList().getMeasurements().begin();
		int ctr = 0;
		for(; itMl != ml->getMeasList().getMeasurements().end(); ++itMl, ++ctr)
		{
			logInfo << " M" << ctr << ":" << (*itMl) << std::endl;
		}
	}


	//========================================
	void onData(const DeviceStatus* const devStat)
	{
		logInfo << std::setw(5) << devStat->getSerializedSize() << " Bytes  "
				<< "DevStat received"
				<< std::endl;
	}

	//========================================
	void onData(const DeviceStatus6303* const devStat)
	{
		logInfo << std::setw(5) << devStat->getSerializedSize() << " Bytes  "
				<< "DevStat 0x6303 received"
				<< std::endl;
	}

	//========================================
	void onData(const LogMessageError* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Error) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	void onData(const LogMessageWarning* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Warning) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	void onData(const LogMessageNote* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Note) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================
	void onData(const LogMessageDebug* const logMsg)
	{
		logInfo << std::setw(5) << logMsg->getSerializedSize() << " Bytes  "
				<< "LogMessage (Debug) received: time: " << logMsg->getTraceLevel() << ": " << logMsg->getMessage() << std::endl;
	}

	//========================================

}; // AllListener

//======================================================================
//======================================================================
//======================================================================

class CustomLogStreamCallbackExample : public CustomLogStreamCallback {
public:
	virtual ~CustomLogStreamCallbackExample() {}
public:
	virtual void onLineEnd(const char* const s, const int)
	{
		std::cerr << s << std::endl;
	}
}; // CustomLogStreamCallback


//======================================================================
//======================================================================
//======================================================================

int checkArguments(const int argc, const char** argv, bool& hasLogFile)
{
	const int minNbOfNeededArguments = 2;
	const int maxNbOfNeededArguments = 3;

	bool wrongNbOfArguments = false;
	if (argc < minNbOfNeededArguments) {
		std::cerr << "Missing argument" << std::endl;
		wrongNbOfArguments = true;
	}
	else if (argc > maxNbOfNeededArguments) {
		std::cerr << "Too many argument" << std::endl;
		wrongNbOfArguments = true;
	}

	if (wrongNbOfArguments) {
		std::cerr << argv[0] << " " << " INPUTFILENAME [LOGFILE]" << std::endl;
		std::cerr << "\tINPUTFILENAME Name of the file to use as input instead of a sensor." << std::endl;
		std::cerr << "\tLOGFILE name of the log file. If ommitted, the log output will be performed to stderr." << std::endl;
		return 1;
	}

	hasLogFile = (argc == maxNbOfNeededArguments);
	return 0;
}

//======================================================================

int main(const int argc, const char** argv)
{
	std::cerr << argv[0] << " Version " << appVersion.toString();
	std::cerr << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;

	bool hasLogFile;
	const int checkResult = checkArguments(argc, argv, hasLogFile);
	if (checkResult != 0)
		exit(checkResult);
	int currArg = 1;

	std::string filename = argv[currArg++];

	const off_t maxLogFileSize = 1000000;

	LogFileManager logFileManager;
	ibeosdk::LogFile::setTargetFileSize(maxLogFileSize);

	if (hasLogFile) {
		ibeosdk::LogFile::setLogFileBaseName(argv[currArg++]);
	}
	const ibeosdk::LogLevel ll = ibeosdk::logLevelFromString("Debug");
	ibeosdk::LogFile::setLogLevel(ll);

	static CustomLogStreamCallbackExample clsce;

	if (!hasLogFile)
		LogFile::setCustomLogStreamCallback(&clsce);

	logFileManager.start();

	if (hasLogFile) {
		logInfo << argv[0] << " Version " << appVersion.toString()
		        << "  using IbeoSDK " << ibeoSDK.getVersion().toString() << std::endl;
	}

	file_demo(filename);

	exit(0);
}

//======================================================================

void file_demo(const std::string& filename)
{
	IdcFile file;
	file.open(filename);
	if (file.isOpen()) {
		AllListener allListener;

		file.registerListener(dynamic_cast<DataListener<FrameEndSeparator>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ScanLux>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ScanEcu>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<Scan2208>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListLux>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListEcu>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListScala>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListScala2271>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<ObjectListEcuEt>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<Image>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<PositionWgs84_2604>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<VehicleStateBasicLux>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<VehicleStateBasicEcu2806>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<VehicleStateBasicEcu>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<MeasurementList2821>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<DeviceStatus>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<LogMessageError>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<LogMessageWarning>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<LogMessageNote>*>(&allListener));
		file.registerListener(dynamic_cast<DataListener<LogMessageDebug>*>(&allListener));

		const DataBlock* db = NULL;
		unsigned short nbMessages = 0; // # of messages we parsed

		while (file.isGood()) {
			db = file.getNextDataBlock();
			if (db == NULL) {
				continue; // might be eof or unknown file type
			}
			file.notifyListeners(db);
			++nbMessages;
		}

		logDebug << "EOF reached. " << nbMessages << " known blocks found." << std::endl;
	}
	else {
		logError << "File not readable." << std::endl;
	}
}

//======================================================================
