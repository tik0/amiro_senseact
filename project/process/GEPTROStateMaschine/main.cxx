// Messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// CAN
#include <ControllerAreaNetwork.h>
#include <Types.h>
#include <Constants.h>
types::position homingPosition;
ControllerAreaNetwork myCAN;

// For running system comands
#include <stdlib.h>

// For using kbhit
#include <unistd.h>

// For checking character pressed in the console
#include <kbhit.hpp>

// For using vector
#include <vector>

// For using string
#include <string>

// For program options
#include <boost/program_options.hpp>
// RSB
#include <converter/matConverter/matConverter.hpp>
#include <converter/vecIntConverter/main.hpp>
#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>

using namespace boost;
using namespace std;
using std::vector;
using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;
using namespace po = boost::program_options;

enum states { 
	idle,
	init,
	exploration,
	explorationfinish,
	findblobs,
	findblobsfinish,
	objectdetection,
	objectdetectionfinish,
	waiting,
	objectdelivery,
	objectdeliveryfinish
}

enum objectdetectionstates {
	driving,
	detecting
}

std::string inscope = "/inputamiro";
std::string outscope = "/outputamiro";

int main(int argc, char **argv) {

po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    //("spread,s", po::value < std::string > (&g_sRemoteServer), "IP of remote spread server.")

}
