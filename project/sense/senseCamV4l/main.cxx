#define INFO_MSG_
// #define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <converter/matConverter/matConverter.hpp>
// Video 4 Linux
#include <linux/videodev2.h>
#include <libv4l2.h>
#include <fcntl.h>
// #include <errno.h>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace muroxConverter; // The namespace for the own converters
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
static std::string g_sDevice = "/dev/video0";

int main(int argc, char **argv) {  

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("inscope,i", po::value < std::string > (&g_sInScope),"Scope for receiving compressed images")
            ("device,d", po::value < std::string > (&g_sDevice),"Location of device");

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
    
    INFO_MSG( "Scope: " << g_sInScope)

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Register our converter within the collection of converters for
  // the string wire-type (which is used for arrays of octets in
  // C++).
  shared_ptr<MatConverter> converter(new MatConverter());
  converterRepository<std::string>()->registerConverter(converter);

  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create the informer
  Informer<cv::Mat>::Ptr informer = getFactory().createInformer<cv::Mat> (Scope(g_sInScope));
  ////////////////////////////////////////////////////////////////////////////////////////////////////


        struct v4l2_format fmt;
        int rc, fd = -1;
        unsigned int i, length;
        const char *dev_name = g_sDevice.c_str();
        char out_name[256];
        FILE *fout;
        void *buffer;
        
        fd = v4l2_open(dev_name, O_RDWR, 0);
        if (fd < 0) {
                fprintf(stderr, "Cannot open device %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = 320;  // Maximum width
        fmt.fmt.pix.height      = 240;  // Maximum hight
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
        fmt.fmt.pix.field       = V4L2_FIELD_ANY;
        rc = v4l2_ioctl(fd, VIDIOC_S_FMT, &fmt);
        printf("Resolution: %d x %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
        if (rc == -1) {
                fprintf(stderr, "Error: %d, %s\n", errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
        if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB24) {
                fprintf(stderr, "Error: Libv4l did not accept format\n");
                exit(EXIT_FAILURE);
        }
//      if ((fmt.fmt.pix.width != 320) || (fmt.fmt.pix.height != 240))
//              printf("Warning: driver is sending image at %dx%d\n",
//                      fmt.fmt.pix.width, fmt.fmt.pix.height);

        if (fmt.fmt.pix.sizeimage < (fmt.fmt.pix.width * fmt.fmt.pix.height)) {
                fprintf(stderr, "Error: Driver is sending image at %dx%d with size of %d\n",
                        fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);
                exit(EXIT_FAILURE);
        }

        buffer = malloc(fmt.fmt.pix.sizeimage);
        if (buffer == NULL) {
                fprintf(stderr, "Cannot allocate buffer\n");
                exit(EXIT_FAILURE);
        }


  // Allocate a frame object to store the picture
   shared_ptr<cv::Mat> frame(new cv::Mat( fmt.fmt.pix.height, fmt.fmt.pix.width, CV_8UC3));

  // Process the cam forever
  for (;;) {
//     cv::waitKey(1);
    // Save the actual picture to the frame object
                length = v4l2_read(fd, (void*) frame->data , fmt.fmt.pix.sizeimage);
                if (length == -1) {
                        fprintf(stderr, "Error: %d, %s\n", errno, strerror(errno));
                        exit(EXIT_FAILURE);
                }
        informer->publish(frame);
     

  }

  // Free everything
  free(buffer);
  v4l2_close(fd);

  return 0;

}
