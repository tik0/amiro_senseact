//============================================================================
// Name        : main.cxx
// Author      : mbarther
// Description : Reads the proximity ring sensor data of AMiRo
//============================================================================

#include <irConverter.h>

using namespace std;
using namespace muroxConverter;




void IRConverter::loadIRValues() {
	while(loading) {
		std::vector< uint16_t > irValuesSave(IR_VALUE_COUNT,0);
		int fail = canConnection->getProximityRingValue(irValuesSave);
		if (fail == 0) {
			irValueLock.lock();
			for (int idx=0; idx<IR_VALUE_COUNT; idx++) {
				irValues[idx] = irValuesSave[idx];
			}
			irValueLock.unlock();
		}
		usleep(MAX_IR_VALUE_AGE_MS*1000);
	}
}


IRConverter::IRConverter(ControllerAreaNetwork *CAN) {
	canConnection = CAN;
	for (int idx=0; idx<IR_VALUE_COUNT; idx++) {
		irValues[idx] = 1;
	}
	irOffsets[0] = 0;
	irOffsets[1] = 0;
	irOffsets[2] = 0;
	irOffsets[3] = 0;
	irOffsets[4] = 0;
	irOffsets[5] = 0;
	irOffsets[6] = 0;
	irOffsets[7] = 0;
	loading = true;
	boost::thread t(loadIrValues);
}



bool IRConverter::initializeOffsets() {
	// TODO calculate offsets and read them over CAN
	return true;
}



int IRConverter::getIRValueCount() {
	return IR_VALUE_COUNT;
}



void IRConverter::getIRValues(std::vector<uint16_t> values) {
	irValueLock.lock();
	for (int idx=0; idx<IR_VALUE_COUNT; idx++) {
		values[idx] = irValues[idx];
	}
	irValueLock.unlock(); 
}



double IRConverter::ir2dist(double irValue, double angle, int sensorNo) {
	irValue -= irOffsets[sensorNo];

	// calculate distance
	double cosxi = cos(IR_CONVERTER_CALC_XI*angle);
	if (cosxi < 0) {
		cosxi *= -1;
	}
	double divi = irValue-IR_CONVERTER_CALC_BETA;
	if (divi <= 0) {
		divi = 1;
	}
	return sqrt(IR_CONVERTER_CALC_ALPHA*cosxi/divi + IR_CONVERTER_CALC_DELTA*cosxi);
}



double IRConverter::dist2ir(double dist, double angle, int sensorNo) {
	double cosxi = cos(IR_CONVERTER_CALC_XI*angle);
	return (IR_CONVERTER_CALC_ALPHA*cosxi/(dist*dist - IR_CONVERTER_CALC_DELTA*cosxi) + IR_CONVERTER_CALC_BETA + irOffsets[sensorNo]);
}



double IRConverter::getDistError(double dist, double angle) {
	double sig = sqrt(IR_CONVERTER_CALC_MEAS_VARIANCE);

	// calculate error
	double cosxi = cos(IR_CONVERTER_CALC_XI*angle);
	if (cosxi < 0) {
		cosxi *= -1;
	}
	double diffdistdelta = dist*dist/cosxi - IR_CONVERTER_CALC_DELTA;
	double error = (diffdistdelta*diffdistdelta)/(2*dist*IR_CONVERTER_CALC_ALPHA*sqrt(1/cosxi))*sig;

	return error;
}
