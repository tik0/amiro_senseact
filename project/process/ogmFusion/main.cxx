
#include <opencv2/opencv.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>
#include <utils.h>

using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace rsb::converter;

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

static std::string iWeed = "/image";
static std::string iFloor = "/image";
static std::string iEdge = "/image";
static std::string iCrop = "/image";

int main(int argc, char **argv) {  

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("iWeed", po::value < std::string > (&iWeed),"Scope for receiving weed guesses")
            ("iFloor", po::value < std::string > (&iFloor),"Scope for receiving floor guesses")
            ("iEdge", po::value < std::string > (&iEdge),"Scope for receiving edge guesses")
            ("iCrop", po::value < std::string > (&iCrop),"Scope for receiving crop guesses");

    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(options).positional(p).run(),
            vm);

    // first, process the help option
    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }
    
    // afterwards, let program options handle argument errors
    po::notify(vm);
    
    INFO_MSG( "" )

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create and start the listener
  rsb::ListenerPtr listenerWeed = factory.createListener(iWeed);
  rsb::ListenerPtr listenerFloor = factory.createListener(iFloor);
  rsb::ListenerPtr listenerEdge = factory.createListener(iEdge);
  rsb::ListenerPtr listenerCrop = factory.createListener(iCrop);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > weedQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > floorQueue(
                        new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > edgeQueue(
                        new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > cropQueue(
                        new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

  listenerWeed->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(weedQueue)));
  listenerFloor->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(floorQueue)));
  listenerEdge->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(edgeQueue)));
  listenerCrop->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(cropQueue)));

  bool weedPresent = false;
  bool floorPresent = false;
  bool edgePresent = false;
  bool cropPresent = false;
  // Pop the images and show them
  // TODO handle different resolutions
  cv::Mat globalOgm(1024, 1024, CV_8UC3, cv::Scalar(128, 128, 128));
  cv::Point2f a;
  std::string weedOgm;
  std::string floorOgm;
  std::string edgeOgm;
  std::string cropOgm;

  bool flag = true;
  std::cout << "1" << std::endl;
  while (true) {
    // Get the image as string

    if (!weedQueue->empty()) {weedOgm  = *weedQueue->pop().get(); weedPresent = true;}    else weedPresent = false;
    if (!weedQueue->empty()) {floorOgm = *floorQueue->pop().get(); floorPresent = true;} else floorPresent = false;
    if (!weedQueue->empty()) {edgeOgm  = *edgeQueue->pop().get(); edgePresent = true;}    else edgePresent = false;
    if (!weedQueue->empty()) {cropOgm  = *cropQueue->pop().get(); cropPresent = true;}    else cropPresent = false;
    weedOgm  = *weedQueue->pop().get(); weedPresent = true;
    // Copy to a vector
    if (weedPresent && flag) {
      flag = false;
      std::vector<unsigned char> data(weedOgm.begin(), weedOgm.end());
      // Decode the image
      cv::Mat localOgm = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);

      std::cout << ": " << localOgm.type()  << " " << CV_8UC3 << std::endl << std::flush;
      utils::addLocalToGlobalMap(localOgm, globalOgm, cv::Point2f(1.5,0));
      std::cout << "2" << std::endl;
    }
    // Exit the program if any key was pressed
    if ( cv::waitKey(1) >= 0 )
      break;
    cv::imshow("globalOgm", globalOgm);
    sleep(1);
  }
  return 0;
}
