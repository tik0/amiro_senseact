//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send controls to the motor of AMiRo
//               Velocity are received in mm/s via RSB
//               Angular velocity are received in ��/s via RSB
//============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/MetaData.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

#include <stdint.h>  // int32

#include <ControllerAreaNetwork.h>

#include <mutex>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/EventQueuePushHandler.h>

using namespace std;
using namespace muroxConverter;

struct MotorCmd {
	bool valid;
	int forward_vel;
	int angular_vel;
	boost::uint64_t expiration_time;
};

//#define old

MotorCmd rankedMotorCmdTable[100];
int idxNewCmd = 0, currIdx = -1;
bool newLowerCmd = false;
std::mutex table_mutex;

void printRankedCmdTalbe(string reason) {
	cout << "========= " << left << setw(40) << setfill('=') << reason << endl;
	cout << " Rank | Valid |   Forward   |    Angular   | Remaining time" << endl;
	int maxRow = 5;
	for(int i = 0; i <= maxRow || i == 99; ++i) {
		stringstream ssExpirationTime;
		if (rankedMotorCmdTable[i].valid) {
			switch (rankedMotorCmdTable[i].expiration_time) {
			case 0 : ssExpirationTime << "forever"; break;
			case 1 : ssExpirationTime << 0 << " ms"; break;
			default: ssExpirationTime << int(rankedMotorCmdTable[i].expiration_time - rsc::misc::currentTimeMicros()) / 1000 << " ms";}
		} else ssExpirationTime << "expired";
		cout
		<< (i == currIdx  ? "->" : "  ")
		<< " " << (i < 10 ? "0" : "") << i
		<< " |   " << (rankedMotorCmdTable[i].valid ? 1 : 0)
		<< "   | " << setw(7) << setfill(' ') << rankedMotorCmdTable[i].forward_vel << "  µm"
		<< " | " << setw(7) << setfill(' ') << rankedMotorCmdTable[i].angular_vel << " µrad"
		<< " | " << ssExpirationTime.str() << endl;
		if (i == maxRow) {
			cout << " ...  |  ...  | ...         | ...          | " << "..." << endl;
			i = 98;
		}
	}
}
#ifdef old
void insertMotorCmdFromEvent(rsb::EventPtr motorCmdEvent) {
	boost::shared_ptr<std::vector<int> > motorCmdVec = static_pointer_cast<std::vector<int> >(motorCmdEvent->getData());
	boost::uint64_t duration = motorCmdVec->at(2);
	struct MotorCmd newMotorCmd = {true, motorCmdVec->at(0), motorCmdVec->at(1), duration + motorCmdEvent->getMetaData().getReceiveTime()};
	string behaviorScope = motorCmdEvent->getScopePtr()->getComponents().back();
	int idx = (int(behaviorScope[0])-48) * 10 + (int(behaviorScope[1]) - 48);
	table_mutex.lock();
	rankedMotorCmdTable[idx] = newMotorCmd;
	newLowerCmd = idx <= currIdx;
	idxNewCmd = idx;
	printRankedCmdTalbe("Insertion ");
	table_mutex.unlock();
}
#else
void insertMotorCmdFromEvent(rsb::EventPtr motorCmdEvent) {
	boost::shared_ptr<std::vector<int> > motorCmdVec = static_pointer_cast<std::vector<int> >(motorCmdEvent->getData());
	boost::uint64_t duration = motorCmdVec->at(2);
	struct MotorCmd newMotorCmd = {true, motorCmdVec->at(0), motorCmdVec->at(1), duration == 0 ? 1 : duration + motorCmdEvent->getMetaData().getReceiveTime()};
	string behaviorScope = motorCmdEvent->getScopePtr()->getComponents().back();
	int idx = (int(behaviorScope[0])-48) * 10 + (int(behaviorScope[1]) - 48);
	rankedMotorCmdTable[idx] = newMotorCmd;
	newLowerCmd = idx <= currIdx;
	idxNewCmd = idx;
}
#endif

int main (int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;

  std::string rsbInScope = "/motor";
  uint32_t rsbPeriod = 0;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,i", po::value < std::string > (&rsbInScope), "Scope for receiving the motor steering commands");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  // Create the CAN interface
  ControllerAreaNetwork CAN;

  // Get the RSB factory
  rsb::Factory& factory = rsb::getFactory();

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converterVecInt);

  rsb::ListenerPtr motorCmdListener = factory.createListener(rsbInScope);

#ifdef old

  motorCmdListener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&insertMotorCmdFromEvent)));

  bool cmdInvalid, cmdValid, cmdExpired;

  while (true) {
	  // Check if cmd is valid
	  table_mutex.lock();
	  cmdInvalid = !rankedMotorCmdTable[currIdx].valid;
	  table_mutex.unlock();
	  // Skip cmd if not valid
	  if (!cmdInvalid) {
		  // Update validity
		  table_mutex.lock();
		  cmdValid = !newLowerCmd && rsc::misc::currentTimeMicros() < rankedMotorCmdTable[currIdx].expiration_time;
		  table_mutex.unlock();
		  if (cmdValid) {
			  // Just started with this cmd -> apply it
			  table_mutex.lock();
			  CAN.setTargetSpeed(rankedMotorCmdTable[currIdx].forward_vel, rankedMotorCmdTable[currIdx].angular_vel);
			  printRankedCmdTalbe("New command applied ");
			  table_mutex.unlock();
			  // Cmd keeps applied as long as it is not expired
			  do {
				  table_mutex.lock();
				  cmdExpired = newLowerCmd || rsc::misc::currentTimeMicros() > rankedMotorCmdTable[currIdx].expiration_time || rankedMotorCmdTable[currIdx].expiration_time > rsc::misc::currentTimeMicros() + 1000000000;
				  table_mutex.unlock();
				  boost::this_thread::sleep(boost::posix_time::milliseconds(rsbPeriod));
			  } while (!cmdExpired);
		  }
		  // If loop aborted because cmd was expired -> set invalid
		  table_mutex.lock();
		  if (!newLowerCmd) {
			  rankedMotorCmdTable[currIdx].valid = false;
			  printRankedCmdTalbe("Current command expired ");
		  }
		  table_mutex.unlock();
	  }
	  // If currently active cmd expired -> continue with next one (wrap around)
	  table_mutex.lock();
	  currIdx = ++currIdx % 100;
	  // If execution of current cmd ended because of insertion of new lower level cmd -> jump there
	  if (newLowerCmd) {
		  currIdx = idxNewCmd;
		  newLowerCmd = false;
	  }
	  // Current cmd expired and no valid higher level cmd available -> stop movement
	  if (currIdx == 0)  CAN.setTargetSpeed(0, 0);
	  table_mutex.unlock();
  }

#else

  boost::shared_ptr<rsc::threading::SynchronizedQueue<rsb::EventPtr> > cmdQueue(new rsc::threading::SynchronizedQueue<rsb::EventPtr>(1));
  motorCmdListener->addHandler(rsb::HandlerPtr(new rsb::util::EventQueuePushHandler(cmdQueue)));
  rsb::EventPtr newMotorCmdEvent;

  // Init with stop cmd
  currIdx = 99;
  struct MotorCmd stopCmd = {true, 0, 0, 0};
  rankedMotorCmdTable[currIdx--] = stopCmd; // -1 because of increment at beginning of loop

  boost::uint32_t remainingExecTime;
  while(true) {
	  ++currIdx;
	  // Skip cmd if [not valid] or [valid but not valid after update] (additionally some assignments)
	  if (!rankedMotorCmdTable[currIdx].valid) continue;
	  if (rankedMotorCmdTable[currIdx].valid = (rankedMotorCmdTable[currIdx].expiration_time > rsc::misc::currentTimeMicros())) {
		  remainingExecTime = (rankedMotorCmdTable[currIdx].expiration_time - rsc::misc::currentTimeMicros())/1000;
	  } else {
		  if (rankedMotorCmdTable[currIdx].expiration_time < 2) { // Special meaning of expiration_time (see below)
			  rankedMotorCmdTable[currIdx].valid = true;
			  remainingExecTime = rankedMotorCmdTable[currIdx].expiration_time; // 0 -> wait forever, 1 -> immediately expire
		  } else continue;
	  }
	  // Start execution of currently lowest level and valid cmd
	  CAN.setTargetSpeed(rankedMotorCmdTable[currIdx].forward_vel, rankedMotorCmdTable[currIdx].angular_vel);
	  try { // ...to pop new cmd from queue (if no new cmd available -> blocks until currently active cmd is expired)
		  if (remainingExecTime == 1)
			  throw rsc::threading::QueueEmptyException(); // immediately expire
		  else
			  newMotorCmdEvent = cmdQueue->pop(remainingExecTime);
	  } catch(const rsc::threading::QueueEmptyException& timeoutException){
		  // Popping timed out -> currently active cmd expired
		  rankedMotorCmdTable[currIdx].valid = false;
		  printRankedCmdTalbe("Current command expired ");
		  // Continue with execution of next higher level cmd
		  continue;
	  }
	  // No exception -> popping succeeded -> new cmd came in -> insert in table and check priority
	  insertMotorCmdFromEvent(newMotorCmdEvent);
	  if (newLowerCmd) { // if new cmd has lower priority -> jump to that cmd
		  currIdx = idxNewCmd;
		  newLowerCmd = false;
	  }
	  printRankedCmdTalbe("Insertion ");
	  --currIdx; // -1 because of increment at beginning
	  // otherwise just continue with execution of previously active cmd:
	  // -> currIdx not changed and cmd still valid
	  // -> remaining execution time gets adjusted at beginning of loop
  }

#endif

  return EXIT_SUCCESS;
}
