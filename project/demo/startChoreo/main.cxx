//============================================================================
// Name        : main.cxx
// Author      : fpatzelt <fpatzelt@techfak.uni-bielefeld.de>
// Description : Statemachine to perform a choreography.
//============================================================================

#define SIMULATION

#include <iostream>
#include <string>
#include <zbar.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace cv;
using namespace std;
using namespace zbar;

// boost
#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
namespace po = boost::program_options;
using namespace boost::chrono;

// rsb
#include <rsb/Factory.h>
#include <rsb/Informer.h>
using namespace rsb;

int main(int argc, char **argv) {

	// scopenames for rsb
	std::string choreoOutscope = "/choreo";

	std::string choreoName = "testChoreo.xml";

	// Handle program options
	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
				("name,n",po::value<std::string>(&choreoName),"Name of the choreography.")
				("choreoOut", po::value<std::string>(&choreoOutscope),"Choreography outscope.");

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

	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// prepare RSB informer for choreos
	Informer<string>::Ptr choreoInformer = factory.createInformer<string>(choreoOutscope);

	// Open camera no. 0 and check for success
	VideoCapture cap(0);
	if (!cap.isOpened()) // if not success, exit program  
	{  
		cout << "Cannot open the video cam. Exiting..." << endl;
		return -1;  
	}

	// Create window for camera stream
	namedWindow("Videostream",CV_WINDOW_AUTOSIZE);

	Mat img, augmented;

	// Create zbar image scanner
	ImageScanner scanner;
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

	bool codeFound = false;

	// Endlessly read image from camera
	while(!codeFound)
	{
		if (!cap.read(img)) //if not success, break loop
		{  
			cout << "Cannot read frame from camera. Exiting..." << endl;
			break;  
		}

		// Wrap image data
		augmented = img.clone(); // clone before converting
		cvtColor(img,img,CV_BGR2GRAY);
		uchar *raw = (uchar *)img.data;
		int width = img.cols;
		int height = img.rows;
		Image image(width, height, "Y800", raw, width * height);

		// Scan the image for codes
		scanner.scan(image);

		// Extract results
		for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
		{   
			choreoName = symbol->get_data();
			cout << "Decoded " << symbol->get_type_name() << " symbol \"" << symbol->get_data() << '"' <<" "<< endl;
			codeFound = true;

			// Draw rectangle around QR code
			vector<Point> vp;
			int n = symbol->get_location_size();
			for(int i = 0; i < n; i++) vp.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
			RotatedRect r = minAreaRect(vp);
			Point2f pts[4];   
			r.points(pts);   
			for(int i=0;i<4;i++) line(augmented,pts[i],pts[(i+1)%4],Scalar(255,0,0),3);
		}    

		imshow("Videostream",augmented);

		if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop  
		{
			cout << "esc key is pressed by user. Exiting..." << endl;
			break;   
		}  

		// Publish the data
		Informer<string>::DataPtr message(new string(choreoName));
		choreoInformer->publish(message);
	}

	return EXIT_SUCCESS;
}
