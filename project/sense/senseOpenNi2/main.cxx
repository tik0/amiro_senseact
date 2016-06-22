#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <converter/iplImageConverter/IplImageConverter.h>

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

#include <stdio.h>
#include <OpenNI.h>

//#include <OniSampleUtilities.h>

#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

using namespace boost;
using namespace std;
using namespace cv;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;
using namespace rst::converters::opencv;

static std::string g_sOutScope = "/depthImage";

int main(int argc, char **argv) {

   namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for sending images.");

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
    
    INFO_MSG( "Scope: " << g_sOutScope)

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // Register our converter within the collection of converters for
    // the string wire-type (which is used for arrays of octets in
    // C++).
    boost::shared_ptr<IplImageConverter> converter(new IplImageConverter());
    converterRepository<std::string>()->registerConverter(converter);

    rsb::Factory &factory = rsb::getFactory();

    // Create the informer
    //Informer<rst::vision::Image>::Ptr informer = getFactory().createInformer<rst::vision::Image> (Scope(g_sOutScope));
    Informer<IplImage>::Ptr informer = getFactory().createInformer<IplImage> (Scope(g_sOutScope));
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
      printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
      return 1;
    }

    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
      printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
      return 2;
    }

    VideoStream depth;

    if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
      rc = depth.create(device, SENSOR_DEPTH);
      if (rc != STATUS_OK)
      {
        printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        return 3;
      }
    }

    rc = depth.start();
    if (rc != STATUS_OK)
    {
      printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
      return 4;
    }

    VideoFrameRef frame;


  // Allocate a frame object to store the picture
//  boost::shared_ptr<cv::Mat> frame(new cv::Mat);

  while (true)
  {

    int changedStreamDummy;
    VideoStream* pStream = &depth;
    rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
    if (rc != STATUS_OK)
    {
      printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
      continue;
    }

    rc = depth.readFrame(&frame);
    if (rc != STATUS_OK)
    {
      printf("Read failed!\n%s\n", OpenNI::getExtendedError());
      continue;
    }

    if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
    {
      printf("Unexpected frame format\n");
      continue;
    }

    DepthPixel* pDepth = (DepthPixel*)frame.getData();

    int middleIndex = (frame.getHeight()+1)*frame.getWidth()/2;
    printf("[%08llu] %8d\n", (long long)frame.getTimestamp(), pDepth[middleIndex]);

    // Convert to an image which can be send via RSB
    boost::shared_ptr<cv::Mat> image(new cv::Mat(cv::Size(frame.getHeight(), frame.getWidth()), CV_16UC1, (void*)pDepth, cv::Mat::AUTO_STEP));

    boost::shared_ptr<IplImage> sendIplImage(new IplImage(*image));
    informer->publish(sendIplImage);
  }

  depth.stop();
  depth.destroy();
  device.close();
  OpenNI::shutdown();

  return 0;

}
