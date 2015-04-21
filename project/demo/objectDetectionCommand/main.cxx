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
#include <rsb/QueuePushHandler.h>


// protocol defines
std::string COMMAND_QUIT = "ESC";
std::string COMMAND_SAVE = "SAVE";
std::string COMMAND_NEW = "NEW";
std::string COMMAND_DELETE = "DEL";
std::string COMMAND_COMPARE = "COMP";
std::string COMMAND_ENDOFCOMPARE = "END_COMP";
std::string COMMAND_STORE = "STORE";
std::string COMMAND_LOAD = "LOAD";

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

static std::string g_sInScope = "/image";
static std::string g_sOutScope = "/command";

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
    
  INFO_MSG( "Scope: " << g_sInScope);
  INFO_MSG( "Command: " << g_sOutScope);

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create the command informer
  Informer<std::string>::Ptr informer = getFactory().createInformer<std::string> (Scope(g_sOutScope));

  // Create and start the listener
  rsb::ListenerPtr listener = factory.createListener(g_sInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> > > imageQueue(
                      new rsc::threading::SynchronizedQueue<boost::shared_ptr<std::string> >(1));

  listener->addHandler(rsb::HandlerPtr(new rsb::QueuePushHandler<std::string>(imageQueue)));

  // Pop the images and show them
  while (true) {

    // Get the image as string
    std::string imageJpg = *imageQueue->pop().get();
    // Copy to a vector
    std::vector<unsigned char> data(imageJpg.begin(), imageJpg.end());
    // Decode the image
    cv::Mat image = cv::imdecode(data, CV_LOAD_IMAGE_COLOR);
    cv::imshow(g_sInScope, image);

    char key = cv::waitKey(10);
    if (char(key) >= 0) {
      //printf("Key = %i (%c)\n", char(key), char(key));
      std::string command;
      switch (char(key)) {
        case 27:
          command = COMMAND_QUIT;
          break;
        case 's':
          command = COMMAND_SAVE;
          break;
        case 'o':
          command = COMMAND_DELETE;
          break;
        case ' ':
          command = COMMAND_COMPARE;
          break;
        case 'n':
          command = COMMAND_NEW;
          break;
        case 13:
          command = COMMAND_ENDOFCOMPARE;
          break;
        case 'l':
          command = COMMAND_LOAD;
          break;
        default:
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
