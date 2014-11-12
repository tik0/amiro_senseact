//============================================================================
// Name        : main.cxx
// Author      : mbarther
// Description : Reads the proximity ring sensor data of AMiRo
//============================================================================


#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <boost/thread.hpp>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>


using namespace std;
using namespace muroxConverter;

#define MAX_IR_VALUE_AGE_MS 125

#define IR_VALUE_COUNT 8

#define IR_CONVERTER_CALC_ALPHA 0.942693757414292
#define IR_CONVERTER_CALC_BETA -16.252241893638708
#define IR_CONVERTER_CALC_DELTA 0
#define IR_CONVERTER_CALC_XI 1.236518540376969
#define IR_CONVERTER_CALC_MEAS_VARIANCE 7.283035954107597

class IRConverter {

public:
	IRConverter(ControllerAreaNetwork *CAN);
	bool initializeOffsets();
	void loadIRValues();
	int getIRValueCount();
	void getIRValues(std::vector<uint16_t> values);
	double ir2dist(double irValue, double angle, int sensorNo);
	double dist2ir(double dist, double angle, int sensorNo);
	double getDistError(double dist, double angle);

private:
	ControllerAreaNetwork *canConnection;
	uint16_t irValues[IR_VALUE_COUNT];
	double irOffsets[IR_VALUE_COUNT];
	bool loading;
	boost::mutex irValueLock;
};
