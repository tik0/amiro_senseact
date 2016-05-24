//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : This tool is for communication with the find loading station
//               tool.
//============================================================================

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>
// Include own converter
#include <converter/vecIntConverter/main.hpp>


// protocol defines
std::string COMMAND_QUIT = "ESC";
std::string COMMAND_FIND = "FIND";

using namespace boost;
using namespace std;
using namespace muroxConverter;
using namespace rsb;
using namespace rsb::converter;
using namespace rsb::util;

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

static std::string g_sImageScope = "/stationDetection/image";
static std::string g_sInScope = "/stationDetection/detected";
static std::string g_sOutScope = "/stationDetection/command";

int main(int argc, char **argv) {  

  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
            ("inscope,i", po::value < std::string > (&g_sInScope),"Scope for receiving compressed images");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store(
      po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
    std::cout << options << "\n";
    exit(1);
  }
    
  // afterwards, let program options handle argument errors
  po::notify(vm);
    
  INFO_MSG( "Output scope: " << g_sOutScope);
  INFO_MSG( "Input scope: " << g_sInScope);
  INFO_MSG( "Image scope: " << g_sImageScope);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  Factory &factory = getFactory();

  // Register new converter for std::vector<int>
  boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
  converterRepository<std::string>()->registerConverter(converter);

  // Create the command informer
  Informer<std::string>::Ptr informer = getFactory().createInformer<std::string> (Scope(g_sOutScope));

  // Create and start the listener for pictures
  rsb::ListenerPtr imageListener = factory.createListener(g_sImageScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > imageQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

  imageListener->addHandler(HandlerPtr(new QueuePushHandler<std::string>(imageQueue)));

  // Create and start the listener for detections
  rsb::ListenerPtr detectListener = factory.createListener(g_sInScope);
  boost::shared_ptr < rsc::threading::SynchronizedQueue < boost::shared_ptr< std::vector<int> > > >detectQueue(new rsc::threading::SynchronizedQueue< boost::shared_ptr< std::vector<int> > >(1));
  detectListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler< std::vector<int> >(detectQueue)));

  // Pop the images and show them
  while (true) {

    // Get the image as string
    std::string imageJpg = *imageQueue->pop().get();
    // Copy to a vector
    std::vector<unsigned char> data(imageJpg.begin(), imageJpg.end());
    // Decode the image
    cv::Mat image = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
    cv::imshow(g_sInScope, image);

    // Get detection
    if (!detectQueue->empty()) {
      boost::shared_ptr< std::vector<int> > stationPos = boost::static_pointer_cast< std::vector<int> >(detectQueue->pop());
      // check for station detection
      if (stationPos->at(0) == 0) {
        INFO_MSG("Station not detected");
      } else {
        float dist = ((float)stationPos->at(0))/1000000.0;
        float angle = ((float)stationPos->at(1))/1000000.0 * 180.0/M_PI;
	INFO_MSG("Station is at " << dist << "m and " << angle << "°");
      }
    }

    char key = cv::waitKey(10);
    if (char(key) >= 0) {
      //printf("Key = %i (%c)\n", char(key), char(key));
      std::string command;
      switch (char(key)) {
        case 27:
          INFO_MSG("Sending quit command.");
          command = COMMAND_QUIT;
          break;
        case ' ':
          INFO_MSG("Sending find command.");
          command = COMMAND_FIND;
          break;
        default:
          INFO_MSG("Unknown command!");
          command = "";
          break;
      }
      if (command != "") {
        shared_ptr<std::string> StringPtr(new std::string(command));
        informer->publish(StringPtr);
      }
      if (char(key) == 27) {
        break;
      }
    }
  }

  return 0;

}
