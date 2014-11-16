#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <converter/matConverter/matConverter.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>

// For program options
#include <boost/program_options.hpp>


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace muroxConverter; // The namespace for the own converters
using namespace rsb::converter;

static std::string g_sOutScope = "/image";
static int g_iDevice = 0;

int main(int argc, char **argv) {

   namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for sending images.")
            ("device,d", po::value < int > (&g_iDevice),"Number of device.");

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
    
    INFO_MSG( "Scope: " << g_sOutScope)
    INFO_MSG( "Device: " << g_iDevice)
    
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Register our converter within the collection of converters for
  // the string wire-type (which is used for arrays of octets in
  // C++).
  shared_ptr<MatConverter> converter(new MatConverter());
  converterRepository<std::string>()->registerConverter(converter);

  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create the informer
  Informer<cv::Mat>::Ptr informer = getFactory().createInformer<cv::Mat> (Scope(g_sOutScope));
  ////////////////////////////////////////////////////////////////////////////////////////////////////


  // Creating the cam object
  cv::VideoCapture cam;
  // Open the device /dev/video<g_iDevice>
  cam.open(g_iDevice);
  // Allocate a frame object to store the picture
  shared_ptr<cv::Mat> frame(new cv::Mat);

  // Process the cam forever
  for (; ;) {
    // Save the actual picture to the frame object
    cam >> *frame;
    // Send the data.
    informer->publish(frame);

  }

  // Free the cam
  cam.release();

  return 0;

}
