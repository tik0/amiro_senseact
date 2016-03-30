/*
 * Takes one image out of NI and calculates the closest distance for every column, to simulate a laserscan over the whole image 
 * Note: It works because of assumption/fact that the ground plane can not be seen directly from the height of AMiRo in Orbbec Camera
 * To start the program run:  sudo ./<name> -d 1000
 * spread should be running for RSB
 * publishes data at /lidar
 * argument d is delay in msecond at which lidar is published
 *Author: ssharma
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <converter/iplImageConverter/IplImageConverter.h>

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

// RST
#include <rsb/converter/ProtocolBufferConverter.h>
// RST Proto types
#include <types/LocatedLaserScan.pb.h>
#include <rst/geometry/Pose.pb.h>

// For program options
#include <boost/program_options.hpp>


#define INFO_MSG_
#include <MSG.h>

#include <stdio.h>
#include <OpenNI.h>
#include <Eigen/Dense>
#include <math.h>
#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

//using namespace boost;
using namespace std;
using namespace cv;
using namespace rsb;
using namespace rsb::converter;
using namespace openni;
using namespace rst::converters::opencv;

class ReadData: public rsc::threading::PeriodicTask {
public:

    ReadData(rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScanSimulation,
             rsb::Informer<rst::vision::LocatedLaserScan>::Ptr informerLaserSim,
              const unsigned int& ms = 1000) :
             rsc::threading::PeriodicTask(ms) {
               //laserScanSimulation(laserScanSimulation), rsc::threading::PeriodicTask(ms), informerLaserSim(informerLaserSim) {
            
      this->laserScanSimulation = laserScanSimulation ; this->informerLaserSim = informerLaserSim;
      /*
      // driver initialize start
      rc = OpenNI::initialize();
      if (rc != STATUS_OK)
      {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        //return 1;
      }

      rc = device.open(ANY_DEVICE);
      if (rc != STATUS_OK)
      {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        //return 2;
      }

      if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
      {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
          printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
          //return 3;
        }
      }

      rc = depth.start();
      if (rc != STATUS_OK)
      {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
        //return 4;
      }
      
      // driver initialize end
      */
      
      rc = OpenNI::initialize();
      if (rc != STATUS_OK)
      {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        //return 1;
      }

      rc = device.open(ANY_DEVICE);
      if (rc != STATUS_OK)
      {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        //return 2;
      }
      
      if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
      {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc != STATUS_OK)
        {
          printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
          //return 3;
        }
      }
      
  }     

    virtual ~ReadData() {
      /*
      depth.stop();
      */
      depth.destroy();
      device.close();
      OpenNI::shutdown();
      
    }

    virtual void execute() {
    
      // driver initialize start
      
      rc = depth.start();
      if (rc != STATUS_OK)
      {
        printf("Couldn't start the depth stream\n%s\n", OpenNI::getExtendedError());
       //return 4;
      }
      
      // driver initialize end
      int changedStreamDummy;
      VideoStream* pStream = &depth;
      rc = OpenNI::waitForAnyStream(&pStream, 1, &changedStreamDummy, SAMPLE_READ_WAIT_TIMEOUT);
      if (rc != STATUS_OK)
      {
        printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
        return;
      }

      rc = depth.readFrame(&frame);
      if (rc != STATUS_OK)
      {
        printf("Read failed!\n%s\n", OpenNI::getExtendedError());
        return;
      }

      if (frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_1_MM && frame.getVideoMode().getPixelFormat() != PIXEL_FORMAT_DEPTH_100_UM)
      {
        printf("Unexpected frame format\n");
        return;
      }

      DepthPixel* pDepth = (DepthPixel*)frame.getData();

      std::vector< std::vector<float> > distances = simpleDistanceFinder(pDepth,(int) frame.getWidth(),(int) frame.getHeight());
      
      // Reserve data
      laserScanSimulation->mutable_scan_values()->Reserve(distances.size());
      for(int idx = 1; idx <= distances.size(); ++idx)
      {
        laserScanSimulation->mutable_scan_values()->Add(0.0f);
      }
      for(int idx = 0; idx < distances.size(); idx++) 
      {
        // Convert the data from millimeter to meter
        laserScanSimulation->mutable_scan_values()->Set(idx, static_cast<float>( distances.at(idx).at(1) / 1000.0f));
        //INFO_MSG( "Laser: " <<  idx << "  ,  "<< static_cast<float>( distances.at(idx).at(1) / 1000.0f) );
        /*if(idx%7==0)
        {
          INFO_MSG( "Laser: " << distances.at(idx).at(1) );
        }*/
      }
      // Send the data.
      informerLaserSim->publish(laserScanSimulation);
      laserScanSimulation->clear_scan_values();

      //close depth stream
      depth.stop();
    }
    
    std::vector< std::vector<float> > simpleDistanceFinder(DepthPixel* pDepth, int width, int height)
    {
      float degreePerPixelH = fovH/width;
      float degreePerPixelV = fovV/height;
      std::vector< std::vector<float> > distances;
      //iterate over the width of image
      for(int i=0; i < width ; i++)
      {
        //large initial value
        float minDist= 9999.0f;
        for(int j=0; j<height; j++)
        {
          float d = (float) pDepth[i + j*width];
          //TODO replace this if
          if(d != 0.0f)
          {
            if(d < minDist)
            {
              minDist = d;
            }
          }
          
        } 
        //convert angle in terms of laser data (emerging from one point)
        int pixelFromCenter= i - (width/2.0f) ;
        float angle = pixelFromCenter*degreePerPixelH;
        std::vector<float> v;
        v.push_back(angle);
        v.push_back(minDist);
        distances.push_back(v);
      }  
      return distances;
    }


private:
    rsb::Informer<rst::vision::LocatedLaserScan>::Ptr informerLaserSim;
    rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScanSimulation;
    
    Status rc;
    VideoStream depth;
    Device device;
    VideoFrameRef frame;
    float fovH = 60.0f;
    float fovV = 49.5f;
};


int main(int argc, char **argv) {

    namespace po = boost::program_options;
    std::string outScope = "/lidar";
   
    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("outscope,o", po::value < std::string > (&outScope),"Scope for sending lidar data.")
            ("lidarPublishDelay,d", po::value<unsigned int>(),"Rate for publishing laser data in ms");

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
    
    //rsc::threading::PeriodicTask(ms);
    unsigned int intervalMs = vm["lidarPublishDelay"].as<unsigned int>();

    rsb::Factory &factory = rsb::getFactory();
    
    // Register 
    boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
    rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);
    // Create the informer
    rsb::Informer<rst::vision::LocatedLaserScan>::Ptr informerLaserSim = factory.createInformer<rst::vision::LocatedLaserScan> (outScope);
    rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScanSimulation(new rst::vision::LocatedLaserScan);
    
    // Field of View ASUS XTION PRO
    //58 H, 45 V, 70 D (Horizontal, Vertical, Diagonal)
    //ORBBEC
    //60 horiz x 49.5 vert. (73 diagonal)
    float fovH = 60.0f;
    float fovV = 49.5f;
    //TODO remove these hard coded values
    float degreePerPixelH = fovH/((int) 320);
    float degreePerPixelV = fovV/((int) 240);

    const int scanSkip=1;
    //const double radPerStep = (360.0 /*°*/ / 1024.0 /*Steps*/) * (M_PI /*rad*/ / 180.0f /*°*/);
    const double radPerStep = (degreePerPixelH) * (M_PI /*rad*/ / 180.0f /*°*/);
    const double radPerSkipStep = (degreePerPixelH) * (M_PI /*rad*/ / 180.0f /*°*/) * scanSkip;
    const double startAngle =  -(fovH/2) * M_PI / 180.0f;
    const double endAngle =  (fovH/2) * M_PI / 180.0f;
    laserScanSimulation->set_scan_angle(fovH - radPerSkipStep);
    laserScanSimulation->set_scan_angle_start(startAngle); //- radPerSkipStep / 2.0f);
    laserScanSimulation->set_scan_angle_end(endAngle);//  + radPerSkipStep / 2.0f);
    laserScanSimulation->set_scan_values_min(0.001); 
    laserScanSimulation->set_scan_values_max(10.0);
    laserScanSimulation->set_scan_angle_increment(radPerSkipStep);
    
    rsc::threading::ThreadedTaskExecutor exec;
    exec.schedule( rsc::threading::TaskPtr( new ReadData( laserScanSimulation,informerLaserSim, intervalMs ) ) );
    while (true) 
    {
        boost::this_thread::sleep(boost::posix_time::seconds(1000));
    }
  return 0;

}

/*rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr driver(rsb::Informer<rst::vision::LocatedLaserScan>::DataPtr laserScanSimulation)
{return laserScanSimulation;
}*/