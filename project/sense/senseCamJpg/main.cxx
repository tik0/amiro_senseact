#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>

// For checking character pressed in the console
#include <kbhit.hpp>

using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
// using namespace muroxConverter;
using namespace rsb::converter;


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

#include <jpeglib.h>

static std::string g_sOutScope = "/image";
static int g_iDevice = 0;
static unsigned int g_uiQuality = 85;

// Depricated and no longer used. Use opencv functions instead
int v4l2_compress_jpeg(int width, int height, unsigned char *src, int src_size, unsigned char *dest, int dest_size);

int main(int argc, char **argv) {  
  
   namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&g_sOutScope),"Scope for sending images")
            ("device,d", po::value < int > (&g_iDevice),"Number of device")
            ("quality,q", po::value < unsigned int > (&g_uiQuality),"Quality of JPEG compression [0 .. 100]");

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
    INFO_MSG( "JPEG Quality: " << g_uiQuality)

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create the informer
  Informer<std::string>::Ptr informer = getFactory().createInformer<std::string> (Scope(g_sOutScope));
  ////////////////////////////////////////////////////////////////////////////////////////////////////


  // Creating the cam object
  cv::VideoCapture cam;
  // Open the device /dev/video<g_iDevice>
  if ( cam.open(g_iDevice) ) {
    // Allocate a frame object to store the picture
    cv::Mat frame;
    // Buffer to store the compressed image
    
    
    // Process the cam forever
    for (; ;) {

      // Save the actual picture to the frame object
      cam >> frame;
      
      // Ask for any keystroke, to quit the capturing
      // Info: cv::waitKey() is useless here, because it wont work in a terminal
      if( kbhit() ) {
//         int KB_code = getchar();
//         cout << "KB_code = " << KB_code << endl;
        break;
      }

      // Compress image
//        v4l2_compress_jpeg(frame->cols, frame->rows, (unsigned char*) frame->data, frame->cols*frame->rows*3, (unsigned char*) &((*frameJpg)[0]), frameJpgSize);
      // Set compression parameters
      vector<uchar> buf;
      vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
      compression_params.push_back(g_uiQuality);
      
       imencode(".jpg", frame, buf, compression_params);
//        cv::imshow("Camera",*frame);
//        cv::waitKey(10);

      // Send the data.
      shared_ptr<std::string> frameJpg(new std::string(buf.begin(), buf.end()));
      informer->publish(frameJpg);

    }
  }

  // Free the cam
  cam.release();

  return 0;

}

int v4l2_compress_jpeg(int width, int height, unsigned char *src, int src_size, unsigned char *dest, int dest_size) {
    struct jpeg_compress_struct cinfo;
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    unsigned char *buf = dest;
    unsigned long buf_size = dest_size;
    int row_stride;

    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);

    cinfo.image_width = width;
    cinfo.image_height = height;
    cinfo.input_components = 3;  // Color depth is RGB

    cinfo.in_color_space = JCS_RGB;

    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, g_uiQuality, FALSE);

    jpeg_mem_dest(&cinfo, &buf, &buf_size);

    jpeg_start_compress(&cinfo, TRUE);

    row_stride = cinfo.image_width * cinfo.input_components;

    while (cinfo.next_scanline < cinfo.image_height) {
        row_pointer[0] = &src[cinfo.next_scanline * row_stride];
        jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }

    jpeg_finish_compress(&cinfo);

    jpeg_destroy_compress(&cinfo);

    if (buf != dest) {
        ERROR_MSG("Destination memory to small (dest: "<< dest << dest_size << " new: " << buf << buf_size)
        return 0;
    }

    return buf_size;
}
