//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : -
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

// inverse sensor model constants
#define IR_CONVERTER_CALC_ALPHA 0.942693757414292
#define IR_CONVERTER_CALC_BETA -16.252241893638708
#define IR_CONVERTER_CALC_DELTA 0
#define IR_CONVERTER_CALC_XI 1.236518540376969
#define IR_CONVERTER_CALC_MEAS_VARIANCE 20.886268537074187 //7.283035954107597

// estimation correction
#define CORRECTION_RANGESTART 0.01
#define CORRECTION_RANGEEND 0.06
#define CORRECTION_DISTSTART 0.01
#define CORRECTION_DISTGRADIANT 0.02 //0.069441
#define CORRECTION_ANGLESTART 0.0713
#define CORRECTION_ANGLEGRADIANT 0.805021 //0.0205021
#define CORRECTION_SIDEMEASSTART 0.004878
#define CORRECTION_SIDEMEASGRADIANT 0.67324


// Guide-Follower protocol
#define PROTOCOL_ERROR 0
#define PROTOCOL_OK 1
#define PROTOCOL_DISCONNECT 2

// state defines
#define STATE_WATCHING 0
#define STATE_TURNING  1
#define STATE_FOLLOW   2


#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>


using namespace rsb;

#include <rst/geometry/Pose.pb.h>
#include <rst/geometry/Translation.pb.h>
#include <rst/geometry/Rotation.pb.h>
using namespace rst::geometry;

#include <converter/vecIntConverter/main.hpp>
//#include <converter/matConverter/matConverter.hpp>
using namespace muroxConverter;

using namespace std;

#include <Types.h>

#include <types/twbTracking.pb.h>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>
#include <extspread.hpp>

using namespace rsb;
using namespace rsb::patterns;

// commands
std::string COMMAND_START = "START";
std::string COMMAND_STOP = "STOP";
std::string COMMAND_QUIT = "QUIT";

// radius of the AMiRo in m
float amiroRadius = 0.05;

// number of sensors
int numSensors = 8;
// maximal range in m
float maxSensorRange = 0.175;

// following constants
float followMinDist = 0.04; // meters
float followMinDistSide = 0.09; // meters
float followDistSlowingDown = 0.1; // meters
int forwardSpeed = 8; // cm/s
int forwardMinSpeed = 3; // cm/s
int turningSpeed = 60; // cradian/s
int turnCorrectSpeed = 25; //cradian/s
float rotationTolarence = M_PI/36.0; // rad

// scopenames for rsb
std::string proxSensorInscope = "/rir_prox/obstacle";
std::string commandInscope = "/follow/proximitysensors";


// method prototypes
float invSensorModel(float angle, float sensorValue);
float getDistErrorSensorModel(float dist, float angle);
void motorActionMilli(int speed, int turn, ControllerAreaNetwork &CAN);
bool referenceRobotIsNear(boost::shared_ptr<std::vector<int>> sensorValues);
std::vector<int> focusReferenceRobot(boost::shared_ptr<std::vector<int>> sensorValues, ControllerAreaNetwork &CAN, int focusStart, int focusEnd);
int followingProc(boost::shared_ptr<std::vector<int>> sensorValues, ControllerAreaNetwork &CAN);
float distCorrection(float sideDist);
float angleCorrection(float sideDist);



int main(int argc, char **argv) {

	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			     ("proximityScope,p", po::value<std::string>(&proxSensorInscope), "Scope for receiving proximity sensor values.")
			     ("commandScope,c", po::value<std::string>(&commandInscope), "Scope for receiving commands.");

	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
		std::cout << options << "\n";
		exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);

	INFO_MSG("Initialize RSB");

	boost::shared_ptr<twbTracking::proto::Pose2D> finalePos(new twbTracking::proto::Pose2D());

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// ------------ Converters ----------------------

	// Register new converter for std::vector<int>
	boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
	rsb::converter::converterRepository<std::string>()->registerConverter(converterVecInt);

	// ------------ Listener ----------------------

	// prepare RSB listener for the IR data
	rsb::ListenerPtr proxListener = factory.createListener(proxSensorInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>> >proxQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::vector<int>>>(1));
	proxListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::vector<int>>(proxQueue)));

	// Create and start the command listener
	rsb::ListenerPtr listener = factory.createListener(commandInscope);
	boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > commandQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
	listener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<std::string>(commandQueue)));


	// Init the CAN interface
	ControllerAreaNetwork myCAN;

	int state = 0;
	bool warningWritten = false;
	bool continueWritten = false;
	bool waitingWritten = false;
	bool continueFollowing = false;
	boost::shared_ptr<std::vector<int>> sensorValues;
	int proxCounter = 0;

	while (true) {
		if (!proxQueue->empty()) {
			// read proximity values
			proxCounter = 0;
			sensorValues = boost::static_pointer_cast<std::vector<int> >(proxQueue->pop());

			if (referenceRobotIsNear(sensorValues) && continueFollowing) {
				if (!continueWritten) {
					continueWritten = true;
					INFO_MSG("Start following.");
				}
				waitingWritten = false;
				warningWritten = false;
				followingProc(sensorValues, myCAN);
			} else if (continueFollowing) {
				if (!warningWritten) {
					warningWritten = true;
					WARNING_MSG("No reference robot!");
				}
				continueWritten = false;
				waitingWritten = false;
				motorActionMilli(0, 0, myCAN);
			} else {
				if (!waitingWritten) {
					waitingWritten = true;
					INFO_MSG("Waiting for follow command ...");
				}
				continueWritten = false;
				warningWritten = false;
				motorActionMilli(0, 0, myCAN);
				usleep(500000);
			}
		} else {
			if (proxCounter >= 4) {
				ERROR_MSG("Didn't received actual sensor values for more than 400 ms!");
				return 1;
			}
			usleep(100000);
			proxCounter++;
		}

		if(!commandQueue->empty()) {
			std::string command = *commandQueue->pop().get();
			if (command == COMMAND_START) {
				continueFollowing = true;
			} else if (command == COMMAND_STOP) {
				continueFollowing = false;
			} else if (command == COMMAND_QUIT) {
				break; // while loop
			} else {
				WARNING_MSG("Received unknown command.");
			}
		}
	}

	INFO_MSG("Closing following behavior.");

	return EXIT_SUCCESS;
}

void motorActionMilli(int speed, int turn, ControllerAreaNetwork &CAN) {
 	CAN.setTargetSpeed(speed*10000, turn*10000);
}

bool referenceRobotIsNear(boost::shared_ptr<std::vector<int>> sensorValues) {
	bool itIs = false;
	for (int idx=0; idx < numSensors; idx++) {
		if (invSensorModel(0, sensorValues->at(idx)) < maxSensorRange) {
			itIs = true;
			break;
		}
	}
	return itIs;
}

std::vector<int> focusReferenceRobot(boost::shared_ptr<std::vector<int>> sensorValues, ControllerAreaNetwork &CAN, int focusStart, int focusEnd) {
	std::vector<int> result (2,0);	
	int idxs[8];
	int idxsCount = 0;
	for (int idx=focusStart; idx < focusEnd; idx++) {
		if (invSensorModel(0, sensorValues->at(idx)) < maxSensorRange) {
			idxs[idxsCount] = idx;
			idxsCount++;
		}
	}
	if (idxsCount > 0) {
		int left;
		switch (idxsCount) {
			case 1:
				left = idxs[0];
				break;
			default:
				if (idxs[0] == 0 && idxs[idxsCount-1] == numSensors-1) {
					left = numSensors-1;
				} else {
					left = numSensors;
					int startIdx = idxs[0];
					for (int idx=1; idx < idxsCount; idx++) {
						if (abs(idxs[idx]-startIdx) == 1) {
							left = startIdx;
							break;
						} else {
							startIdx = idxs[idx];
						}
					}
				} 
		}
		result.at(0) = left;
		result.at(1) = idxsCount;
	}
	return result;
}

int followingProc(boost::shared_ptr<std::vector<int>> sensorValues, ControllerAreaNetwork &CAN) {
	std::vector<int> val = focusReferenceRobot(sensorValues, CAN, 3, 5);
	int left = val.at(0);
	int count = val.at(1);
	if ((left == 3 && count == 2) || ((left == 3 || left == 4) && count == 1)) {
		// calculate distance and angle to reference robot
		float dist, angle;
		if (left == 3 && count == 2) {
			// if both front sensors have refernce robot in range
			float lDist = invSensorModel(0, sensorValues->at(left));
			float rDist = invSensorModel(0, sensorValues->at(left+1));
			float lSide = invSensorModel(0, sensorValues->at(left-1));
			float rSide = invSensorModel(0, sensorValues->at(left+2));
			float posX = (lDist*cos(M_PI/8.0) + rDist*cos(-M_PI/8.0))/2.0;
			float posY = (lDist*sin(M_PI/8.0) + rDist*sin(-M_PI/8.0))/2.0;
			dist = sqrt(posX*posX + posY*posY) - amiroRadius/2.0;
			dist += distCorrection(lSide) + distCorrection(rSide);
			angle = atan(posY/posX);
			angle += angleCorrection(lSide) - angleCorrection(rSide);
		} else {
			float sideDist;
			if (left == 3) {
				sideDist = invSensorModel(0, sensorValues->at(left-1));
				angle = M_PI/8.0;
				angle += angleCorrection(sideDist);
			} else {
				sideDist = invSensorModel(0, sensorValues->at(left+1));
				angle = - M_PI/8.0;
				angle -= angleCorrection(sideDist);
			}
			dist = invSensorModel(0, sensorValues->at(left));
			dist += distCorrection(sideDist);
		}

		// drive towards reference robot
		float alpha = angle;
		float beta = M_PI/2 - angle;
		float drivingRadius = dist*sin(beta)/sin(alpha);
		float drivingDist = 2*drivingRadius*M_PI * alpha/(2*M_PI);

		float rotationSpeed_N = (float)forwardSpeed/100*-alpha/drivingDist;
		int rotationSpeed = rotationSpeed_N*100;

		if ((dist > followMinDist && count == 2) || dist > followMinDistSide) {
			int speed = forwardSpeed;
			if (dist < followMinDist+followDistSlowingDown && count == 2) {
				float part = (dist-followMinDist)/followDistSlowingDown;
				speed = forwardMinSpeed + (int)((forwardSpeed-forwardMinSpeed)*part*part); //sqrt(part*part*part));
			} else if (dist < followMinDistSide+followDistSlowingDown) {
				float part = (dist-followMinDistSide)/followDistSlowingDown;
				speed = forwardMinSpeed + (int)((forwardSpeed-forwardMinSpeed)*part*part); //sqrt(part*part*part));
			}
			motorActionMilli(speed, rotationSpeed, CAN);
		} else if (abs(angle) > rotationTolarence/2.0 && count == 2) {
			if (rotationSpeed > 0) {
				rotationSpeed = min(rotationSpeed, turnCorrectSpeed);
			} else {
				rotationSpeed = max(rotationSpeed, -turnCorrectSpeed);
			}
			motorActionMilli(0, rotationSpeed, CAN);
		} else if (count == 1) {
			motorActionMilli(0, 2*(3.5-left)*turningSpeed, CAN);
		} else {
			motorActionMilli(0, 0, CAN);
			return STATE_WATCHING;
		}

		return STATE_FOLLOW;

	} else {
		motorActionMilli(0, 0, CAN);
		return STATE_WATCHING;
	}
}




// calculate the distance to an obstacle given the obstacles angle relative to the sensor and the sensor value
float invSensorModel(float angle, float sensorValue) {

	float cosxi = cos(IR_CONVERTER_CALC_XI * angle);
	if (cosxi < 0) {
		cosxi *= -1;
	}
	float divi = sensorValue - IR_CONVERTER_CALC_BETA;
	if (divi <= 0) {
		divi = 1;
	}
	return sqrt(
	IR_CONVERTER_CALC_ALPHA * cosxi / divi + IR_CONVERTER_CALC_DELTA * cosxi);
}

float getDistErrorSensorModel(float dist, float angle) {
	float sig = sqrt(IR_CONVERTER_CALC_MEAS_VARIANCE);

	// calculate error
	float cosxi = cos(IR_CONVERTER_CALC_XI * angle);
	if (cosxi < 0) {
		cosxi *= -1;
	}
	float diffdistdelta = dist * dist / cosxi - IR_CONVERTER_CALC_DELTA;
	float error = (diffdistdelta * diffdistdelta) / (2 * dist * IR_CONVERTER_CALC_ALPHA * sqrt(1 / cosxi)) * sig;

	return error;
}



float distCorrection(float sideDist) {
	if (sideDist <= CORRECTION_RANGEEND) {
		float fac = CORRECTION_RANGEEND-sideDist+CORRECTION_RANGESTART;
		return fac*CORRECTION_DISTGRADIANT+CORRECTION_DISTSTART;
	} else {
		return 0.0;
	}
}

float angleCorrection(float sideDist) {
	if (sideDist <= CORRECTION_RANGEEND) {
		float fac = CORRECTION_RANGEEND-sideDist+CORRECTION_RANGESTART;
		return fac*CORRECTION_ANGLEGRADIANT+CORRECTION_ANGLESTART;
	} else {
		return 0.0;
	}
}
