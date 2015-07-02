//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Gets the robot position and sensor values over RSB,
//               updates the occupancy grid map and send it over RSB
//============================================================================

//#define TRACKING
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>
#include <functional>
#include <algorithm>
#include <math.h>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

using namespace std;

int main(int argc, char **argv) {

	FILE *file;
	file = fopen("RingProximityAndOdometry.txt", "w");
	for (int step=0; step<8; step++) {
		fprintf(file, "PRV %i\t", step);
	}
	fprintf(file, "X [ym]\tY [ym]\tTheta [yrad]\n");
	fclose(file);

        INFO_MSG("File has been initialized");

	return EXIT_SUCCESS;
}
