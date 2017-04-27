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

//RSC
#include <rsc/misc/SignalWaiter.h>
//Protoconverter
#include <rsb/converter/ProtocolBufferConverter.h>
//Prototypes
#include <rst/vision/Image.pb.h>

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
	po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
		std::cout << options << "\n";
		exit(1);
	}
	// afterwards, let program options handle argument errors
	po::notify(vm);

	INFO_MSG( "Scope: " << g_sOutScope)
	INFO_MSG( "Device: " << g_iDevice)

	////////////////////////////////////////////////////////////////////////////////////////////////////
//	Register our converter within the collection of converters for
//	the string wire-type (which is used for arrays of octets in C++).
//	shared_ptr<MatConverter> converter(new MatConverter());
//	converterRepository<std::string>()->registerConverter(converter);

	// Register new converter for std::vector<int>
	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::Image> >
		converter(new rsb::converter::ProtocolBufferConverter<rst::vision::Image>());
		rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	rsb::Factory &factory = rsb::getFactory();

	// Create the informer
	Informer<rst::vision::Image>::Ptr informer = getFactory().createInformer<cv::Mat> (Scope(g_sOutScope));
//	Informer<cv::Mat>::Ptr informer = getFactory().createInformer<cv::Mat> (Scope(g_sOutScope));
	////////////////////////////////////////////////////////////////////////////////////////////////////


	// Creating the cam object
	cv::VideoCapture cam;
	//int ex = static_cast<int>(cam.get(CV_CAP_PROP_FOURCC));
	//Size S = Size((int) cam.get(CV_CAP_PROP_FRAME_WIDTH),    // Acquire input size
	//							(int) cam.get(CV_CAP_PROP_FRAME_HEIGHT));
	////Transform from int to char via Bitwise operators
	//char EXT[] = {(char)(ex & 0XFF),(char)((ex & 0XFF00) >> 8),(char)((ex & 0XFF0000) >> 16),(char)((ex & 0XFF000000) >> 24),0};
	//INFO_MSG("Input frame resolution: Width=" << S.width << "  Height=" << S.height)
	//INFO_MSG("Get video frame with following FOURCC code: " << EXT )

	cam.set(CV_CAP_PROP_CONVERT_RGB, static_cast<double>(0));

	// Open the device /dev/video<g_iDevice>
	cam.open(g_iDevice);
	// Allocate a frame object to store the picture
//	shared_ptr<cv::Mat> frame(new cv::Mat);
	cv::Mat frame;

	// Process the cam forever
	while ( true/* TODO in rsb0.12 rsc::misc::lastArrivedSignal() == rsc::misc::Signal::NO_SIGNAL*/) {
		// Save the actual picture to the frame object
		cam >> frame;
		boost::shared_ptr<rst::vision::Image> avatarImage(new rst::vision::Image());
		avatarImage->set_data(frame.data);
		avatarImage->set_width(frame.width);
		avatarImage->set_height(frame.height);
		// Send the data.
		informer->publish(avatarImage);
	}

	// Free the cam
	cam.release();

	return 0/* TODO in rsb0.12 rsc::misc::lastArrivedSignal()*/;

}
