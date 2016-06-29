//============================================================================
// Name        : main.cxx
// Author      : mbarther <mbarther@techfak.uni-bielefeld.de>
// Description : -
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/nonfree/nonfree.hpp>
//#include <converter/iplImageConverter/IplImageConverter.h>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>

#include <rsb/Informer.h>
#include <rsb/Version.h>
#include <rsc/threading/PeriodicTask.h>
#include <rsc/threading/ThreadedTaskExecutor.h>
#include <rsc/misc/SignalWaiter.h>

// RST
//#include <rsb/converter/ProtocolBufferConverter.h>
// RST Proto types
//#include <types/LocatedLaserScan.pb.h>
//#include <rst/geometry/Pose.pb.h>

// For program options
#include <boost/program_options.hpp>


#define INFO_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <stdio.h>
#include <OpenNI.h>
#include <Eigen/Dense>
#include <math.h>
#include <ctime>
#include <boost/chrono.hpp>
#include <boost/chrono/chrono_io.hpp>
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

//using namespace boost;
using namespace std;
using namespace boost::chrono;
using namespace cv;
using namespace boost;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;
//using namespace rst::converters::opencv;


// scope names
std::string rgbScope = "/images/rgb";
std::string depthScope = "/images/depth";
static int g_iDevice = 6;
static unsigned int g_uiQuality = 100;
bool sendImage = false;

#include <jpeglib.h>
#include <libv4l2.h>
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
	cinfo.input_components = 3;

	cinfo.in_color_space = JCS_RGB;

	jpeg_set_defaults(&cinfo);
	jpeg_set_quality(&cinfo, 85, FALSE);

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
		ERROR_MSG("Destination memory to small (dest: " << dest << "x" << dest_size << ", new: " << buf << "x" << buf_size << ")");
		return 0;
	}

	return buf_size;
}

int main(int argc, char **argv) {

	namespace po = boost::program_options;
	std::string outScope = "/lidar";
	std::size_t intervalMs = 1000;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
			("RGBOutscope,r", po::value < std::string > (&rgbScope),"Scope for sending RGB image.")
			("DepthOutscope,d", po::value < std::string > (&depthScope),"Scope for sending depth image.")
			("period,p", po::value < std::size_t > (&intervalMs),"Period for publishing laser data in ms")
			("printTime,t","Prints the time")
			("device,v", po::value < int > (&g_iDevice),"Camera device ID")
			("compression,c", po::value < unsigned int > (&g_uiQuality),"Compression value [0,100]")
			("sendImage,s", "Flag if the image shall be converted to OpenCV and send via RSB.");

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
	
	sendImage = vm.count("sendImage");

	// Initialize clock
	system_clock::time_point systime1, systime2;
	milliseconds mstime;

	if (vm.count("printTime")) systime1 = system_clock::now();
	if (vm.count("printTime")) DEBUG_MSG("Starting RSB Initialization");

	// +++++ RSB Initialization +++++
	rsb::Factory &factory = rsb::getFactory();
	
	INFO_MSG("RSB Scopes:");
	INFO_MSG(" -> sending images:     " << rgbScope);
	INFO_MSG(" -> sending laser data: " << depthScope);
	INFO_MSG("");

	// +++ Informer +++
	// create informer for RGB images
	rsb::Informer<std::string>::Ptr RGBInformer = factory.createInformer<std::string> (rgbScope);
	// create informer for depth images
	rsb::Informer<std::string>::Ptr DepthInformer = factory.createInformer<std::string> (depthScope);


	if (vm.count("printTime")) systime2 = system_clock::now();
	if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
	if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

	if (vm.count("printTime")) systime1 = system_clock::now();
	if (vm.count("printTime")) DEBUG_MSG("Starting OpenNI Initialization");
	// +++++ OpenNI Initialization +++++
	VideoFrameRef rgbImage, depthImage;
	VideoStream rgbStream, depthStream;
	Device device;

	Status respON = OpenNI::initialize();
	if (respON != STATUS_OK) {
		ERROR_MSG("Initialize failed: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}

	Mat rgbMat, depthMat;
	// Compress data
	vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(g_uiQuality);
	vector<uchar> bufRgb;
//	unsigned char bufRgbP[3*height*width];

//	for (;;) {

		respON = device.open(ANY_DEVICE);
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not open device: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}

		// get rgb stream
		const SensorInfo* sinfo = device.getSensorInfo(SENSOR_COLOR);
		if (sinfo != NULL) {
			respON = rgbStream.create(device, SENSOR_COLOR);
			if (respON != STATUS_OK) {
				ERROR_MSG("Could not create RGB stream: " << OpenNI::getExtendedError());
				return EXIT_FAILURE;
			}
			const openni::Array<VideoMode>& modes = sinfo->getSupportedVideoModes();
			rgbStream.setVideoMode(modes[0]);
		}
		

//		VideoMode videoMode = rgbStream.getVideoMode();
//		videoMode.setFps(1);
//		rgbStream.setVideoMode(videoMode);

		respON = rgbStream.start();
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not start the RGB stream: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}

		VideoStream* rgbStreamTemp = &rgbStream;
		int changedRgbStreamDummy;
		respON = OpenNI::waitForAnyStream(&rgbStreamTemp, 1, &changedRgbStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);

		if (respON != STATUS_OK) {
			ERROR_MSG("Wait for RGB stream failed (timeout is "  << SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}

		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");


//		if (vm.count("printTime")) systime1 = system_clock::now();
//		if (vm.count("printTime")) DEBUG_MSG("Starting Loop Initialization");

	//	INFO_MSG("Starting Loop");
//		if (vm.count("printTime")) DEBUG_MSG("-----------------------------------------------")
//		systime1 = system_clock::now();

		respON = rgbStream.readFrame(&rgbImage);
		if (respON != STATUS_OK) {
			ERROR_MSG("Read RGB image failed: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
		int height = rgbImage.getHeight();
		int width = rgbImage.getWidth();
		DEBUG_MSG("Image size: " << width << "/" << height);

/*		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");
		
		if (vm.count("printTime")) systime1 = system_clock::now();
		if (vm.count("printTime")) DEBUG_MSG("Closing stream and device");		
		rgbStream.stop();
		rgbStream.destroy();
		device.close();
		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");
*/		
	for(;;) {

		if (vm.count("printTime")) DEBUG_MSG("-----------------------------------------------")
		systime1 = system_clock::now();
		if (vm.count("printTime")) DEBUG_MSG("Reading camera frame");
		respON = rgbStream.readFrame(&rgbImage);
		if (respON != STATUS_OK) {
			ERROR_MSG("Read RGB image failed: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
		if (vm.count("printTime")) systime2 = system_clock::now();
		if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

		if (!sendImage) {
			int height = rgbImage.getHeight();
			int width = rgbImage.getWidth();
			INFO_MSG(" => Image loaded: " << width << "/" << height);
		} else {
			// Convert to cv::Mat
			if (vm.count("printTime")) systime1 = system_clock::now();
			if (vm.count("printTime")) DEBUG_MSG("Conversion: Prepare Buffer");
			const openni::RGB888Pixel* rgbBuffer = (const openni::RGB888Pixel*)rgbImage.getData();
			if (vm.count("printTime")) systime2 = system_clock::now();
			if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
			if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");
			rgbMat.create(rgbImage.getHeight(), rgbImage.getWidth(), CV_8UC3);
			memcpy(rgbMat.data, rgbBuffer, 3*rgbImage.getHeight()*rgbImage.getWidth()*sizeof(uint8_t));
			if (vm.count("printTime")) systime1 = system_clock::now();
			if (vm.count("printTime")) DEBUG_MSG("Conversion: Convert to RGB");
			cvtColor(rgbMat, rgbMat, CV_BGR2RGB);
			if (vm.count("printTime")) systime2 = system_clock::now();
			if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
			if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");
			if (vm.count("printTime")) systime1 = system_clock::now();
			if (vm.count("printTime")) DEBUG_MSG("Conversion: Flip image");
			flip(rgbMat, rgbMat, 1);
			if (vm.count("printTime")) systime2 = system_clock::now();
			if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
			if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

			// Compress image
			if (vm.count("printTime")) systime1 = system_clock::now();
			if (vm.count("printTime")) DEBUG_MSG("Compressing image");
			imencode(".jpg", rgbMat, bufRgb, compression_params);
	/*		int bufSize = v4l2_compress_jpeg(width, height, resultImage.data, 3*height*width, bufRgbP, 3*height*width);
			bufRgb.clear();
			for (int b=0; b<bufSize; b++) {
				bufRgb.push_back(bufRgbP[b]);
			}
	*/		if (vm.count("printTime")) systime2 = system_clock::now();
			if (vm.count("printTime")) mstime = duration_cast<milliseconds>(systime2-systime1);
			if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms");

			if (vm.count("printTime")) systime1 = system_clock::now();
			if (vm.count("printTime")) DEBUG_MSG("Sending image");
			shared_ptr< std::string > rgbFrameJpg(new std::string(bufRgb.begin(), bufRgb.end()));
			RGBInformer->publish(rgbFrameJpg);
		}
		systime2 = system_clock::now();
		mstime = duration_cast<milliseconds>(systime2-systime1);
		if (vm.count("printTime")) DEBUG_MSG(" -> " << mstime.count() << " ms")
		else DEBUG_MSG("Image sent (" << mstime.count() << " ms)");

		usleep(500000);
	}
	
	rgbStream.stop();
	rgbStream.destroy();
	device.close();
	
	OpenNI::shutdown();


/*
	// get depth image
	if (device.getSensorInfo(SENSOR_DEPTH) != NULL) {
		respON = depthStream.create(device, SENSOR_DEPTH);
		if (respON != STATUS_OK) {
			ERROR_MSG("Could not create depth stream: " << OpenNI::getExtendedError());
			return EXIT_FAILURE;
		}
	}

	respON = depthStream.start();
	if (respON != STATUS_OK) {
		ERROR_MSG("Could not start the depth stream: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}

	VideoStream* depthStreamTemp = &depthStream;
	int changedDepthStreamDummy;
	respON = OpenNI::waitForAnyStream(&depthStreamTemp, 1, &changedDepthStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);

	if (respON != STATUS_OK) {
		ERROR_MSG("Wait for depth stream failed (timeout is "  << SAMPLE_READ_WAIT_TIMEOUT << " ms): " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}

	respON = depthStream.readFrame(&depthImage);
	if (respON != STATUS_OK) {
		ERROR_MSG("Read depth image failed: " << OpenNI::getExtendedError());
		return EXIT_FAILURE;
	}

//	const openni::RGB888Pixel* depthBuffer = (const openni::RGB888Pixel*)depthImage.getData();
//	depthMat.create(depthImage.getHeight(), depthImage.getWidth(), CV_8UC3);
//	memcpy(depthMat.data, depthBuffer, 3*depthImage.getHeight()*depthImage.getWidth()*sizeof(uint8_t));
//	cv::cvtColor(depthMat, depthMat, CV_BGR2RGB);
*/
	


/*
	cv::VideoCapture cam;
//	cv::VideoCapture cam(13);
//	cv::VideoCapture cam(CAP_OPENNI);
	Mat depthImage, bgrImage;

	if (cam.open(g_iDevice)) {
		for(;;) {
			// Get images
			cam.grab();
			cam.retrieve(depthImage, 0);
			cam.retrieve(bgrImage, 5);
	//		cam.retrieve(depthImage, CAP_OPENNI_DEPTH_MAP);
	//		cam.retrieve(bgrImage, CAP_OPENNI_BGR_IMAGE);

			// Compress images
			vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(g_uiQuality);

			// Buffer images
			vector<uchar> bufDepth, bufRgb;
			imencode(".jpg", depthImage, bufDepth, compression_params);
			imencode(".jpg", bgrImage, bufRgb, compression_params);

			// Send images
			shared_ptr< std::string > depthFrameJpg(new std::string(bufDepth.begin(), bufDepth.end()));
			DepthInformer->publish(depthFrameJpg);
			usleep(10000);
			shared_ptr< std::string > rgbFrameJpg(new std::string(bufRgb.begin(), bufRgb.end()));
			RGBInformer->publish(rgbFrameJpg);
			DEBUG_MSG("Image data send")

			usleep(200000);
		}
	} else {
		ERROR_MSG("Camera device " << g_iDevice << " could not be opened!");
	}
*/
	return EXIT_SUCCESS;

}


/*

class ReadData: public rsc::threading::PeriodicTask {
public:

    ReadData(rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScanSimulation,
             rsb::Informer<rst::vision::LocatedLaserScan>::Ptr informerLaserSim,
             bool justCenter,
             const unsigned int& ms = 1000) :
             rsc::threading::PeriodicTask(ms) {

      ///////////////////////////////////////////
      // Configuration
      ///////////////////////////////////////////
      this->laserScanSimulation = laserScanSimulation ; this->informerLaserSim = informerLaserSim;

      this->justCenter = justCenter;
      
      //initialize depthstream
      rc = OpenNI::initialize();
      if (rc != STATUS_OK)
      {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        isValid = false;
      }

      rc = device.open(ANY_DEVICE);
      if (rc != STATUS_OK)
      {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        isValid = false;
      }
      
      if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
      {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
          printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
          isValid = false;
        }
      }

      rc = depth.start();
      if (rc != STATUS_OK)
      {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        isValid = false;
      }
      
      VideoStream* pStreamTemp = &depth;
      int changedStreamDummy;
      rc = OpenNI::waitForAnyStream(&pStreamTemp, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);

      if (rc != STATUS_OK)
      {
        printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
        isValid = false;
      }

      rc = depth.readFrame(&frame);
      if (rc != STATUS_OK)
      {
        printf("Read failed!\n%s\n", OpenNI::getExtendedError());
        isValid = false;
      }

      if (isValid) {
        this->frameWidth = (int) frame.getWidth();
        this->frameHeight = (int) frame.getHeight();
        this->fovH = depth.getHorizontalFieldOfView()*(180/M_PI);
        this->fovV = depth.getVerticalFieldOfView()*(180/M_PI);

        INFO_MSG("Width: " << this->frameWidth << " px")
        INFO_MSG("Height: " << this->frameHeight << " px")
        INFO_MSG("FOV Horizontal: " << this->fovH << " deg")
        INFO_MSG("FOV Vertical: " << this->fovV << " deg")

        this->degreePerPixelH = this->fovH/((float) this->frameWidth);
        this->degreePerPixelV = this->fovV/((float) this->frameHeight);
        const float ORBBEC_ASTRA_S_MIN_RANGE = 0.2; // meter
        const float ORBBEC_ASTRA_S_MAX_RANGE = 5.8; // meter

        const int scanSkip=1;
        //const double radPerStep = (360.0  / 1024.0 ) * (M_PI  / 180.0f);
        const double radPerStep = (degreePerPixelH) * (M_PI / 180.0f);
        const double radPerSkipStep = (degreePerPixelH) * (M_PI / 180.0f) * scanSkip;
        const double startAngle =  -(this->fovH/2) * M_PI / 180.0f;
        const double endAngle =  (this->fovH/2) * M_PI / 180.0f;
        this->laserScanSimulation->set_scan_angle(this->fovH - radPerSkipStep);
        this->laserScanSimulation->set_scan_angle_start(startAngle); //- radPerSkipStep / 2.0f);
        this->laserScanSimulation->set_scan_angle_end(endAngle);//  + radPerSkipStep / 2.0f);
        this->laserScanSimulation->set_scan_values_min(ORBBEC_ASTRA_S_MIN_RANGE);
        this->laserScanSimulation->set_scan_values_max(ORBBEC_ASTRA_S_MAX_RANGE);
        this->laserScanSimulation->set_scan_angle_increment(radPerSkipStep);

        // Reserve data
        this->laserScanSimulation->mutable_scan_values()->Reserve(this->frameWidth);
        for(std::size_t idx = 1; idx <= this->frameWidth; ++idx) {
          this->laserScanSimulation->mutable_scan_values()->Add(0.0f);
        }
      }

  }

    virtual ~ReadData() {
      depth.stop();
      depth.destroy();
      device.close();
      OpenNI::shutdown();
    }

    virtual void execute() {

      if (this->isValid) {
        this->depth.readFrame(&frame);
        pDepth = (DepthPixel*)frame.getData();
        // Get the laser scan
        simpleDistanceFinder();

        // Send the data.
        informerLaserSim->publish(this->laserScanSimulation);
        //this->laserScanSimulation->clear_scan_values();
      } else {
        ERROR_MSG("No valid configuration")
      }
    }

    void simpleDistanceFinder() {

      // iterate over the width of image
      for(std::size_t i=0; i < this->frameWidth ; i++)
      {
        //large initial value
        float minDist= 99999.0f;

        if (justCenter) {
          minDist = (float) this->pDepth[this->frameHeight/2 * this->frameWidth + i];
          if (minDist == 0.0f) {
            minDist = 8.0;
          }
        } else {
          for(std::size_t j=0; j < this->frameHeight; j++) {
            float d = (float) this->pDepth[i + j*this->frameWidth];
            //TODO replace this if
            if(d != 0.0f) {
              if(d < minDist) {
                minDist = d;
              }
            }
          }
        }
        // Convert from mm to m and store the value
        this->laserScanSimulation->mutable_scan_values()->Set(i, float(minDist) / 1000.0f);

//        // Debug output
        //INFO_MSG( "Laser: " <<  i << "  ,  "<< this->laserScanSimulation->mutable_scan_values()->Get(i) );
//        if(i%10==0) {
//          //convert angle in terms of laser data (emerging from one point)
//          int pixelFromCenter= i - (width/2.0f) ;
//          float angle = pixelFromCenter*this->degreePerPixelH;
//          INFO_MSG( "Laser: " << angle <<  " : "<< minDist  << " : " << width << " : " <<height << " " << this->fovH );
//        }
      }  
    }


private:
    rsb::Informer<rst::vision::LocatedLaserScan>::Ptr informerLaserSim;
    rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScanSimulation;
    
    DepthPixel* pDepth = NULL;
    VideoFrameRef frame;
    Status rc;
    VideoStream depth;
    Device device;
    //default values, actual values come from constructor
    // Field of View ASUS XTION PRO
    //58 H, 45 V, 70 D (Horizontal, Vertical, Diagonal)
    //ORBBEC
    //60 horiz x 49.5 vert. (73 diagonal)
    float fovH = 60.0f;  // degree
    float fovV = 49.5f;  // degree
    float degreePerPixelH = 0;  // degree
    float degreePerPixelV = 0;  // degree
    std::size_t frameWidth = 320;  // px
    std::size_t frameHeight = 240;  // px
    bool isValid = true;
    bool justCenter = false;
};


int main(int argc, char **argv) {

    namespace po = boost::program_options;
    std::string outScope = "/lidar";
    std::size_t intervalMs = 1000;
   
    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&outScope),"Scope for sending lidar data")
            ("lidarPublishDelay,d", po::value < std::size_t > (&intervalMs),"Period for publishing laser data in ms")
            ("justCenter,c", "Flag, if not the minimum, but just the vertical centered value shall be used as laser scan.");

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

    rsb::Factory &factory = rsb::getFactory();
    
    // Register 
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
    rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
    // Create the informer
    rsb::Informer<rst::vision::LocatedLaserScan>::Ptr informerLaserSim = factory.createInformer<rst::vision::LocatedLaserScan> (outScope);
    rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScanSimulation(new rst::vision::LocatedLaserScan);

    rsc::threading::ThreadedTaskExecutor exec;
    exec.schedule( rsc::threading::TaskPtr( new ReadData( laserScanSimulation, informerLaserSim, vm.count("justCenter"), intervalMs ) ) );

    rsc::misc::initSignalWaiter();
    return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());

}

*/
