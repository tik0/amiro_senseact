
// std includes
#include <iostream>
using namespace std;



//#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "../../includes/MSG.h"

#include <math.h>

#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/util/QueuePushHandler.h>

// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

// RST Proto types
#include <types/LocatedLaserScan.pb.h>


using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;
using namespace rsb::patterns;


//    if(lparams_.angle_max < lparams_.angle_min){
//      // flip readings
//      for(int i=0; i < scan.scan_values_size(); i++)
//        data.d[i] = (int) (scan.scan_values(scan.scan_values_size()-1-i)*METERS_TO_MM);
//    }else{
//      for(int i=0; i < scan.scan_values_size(); i++)
//        data.d[i] = (int) (scan.scan_values(i)*METERS_TO_MM);
//    }
//
int main(int argc, const char **argv){

  namespace po = boost::program_options;
  std::string lidarInScope = "/AMiRo_Hokuyo/lidar";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("lidarinscope", po::value < std::string > (&lidarInScope), "Scope for receiving lidar data");

  // allow to give the value as a positional argument
  po::positional_options_description p;
  p.add("value", 1);

  po::variables_map vm;
  po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  // Get the RSB factory
  rsb::Factory& factory = rsb::Factory::getInstance();

  // Register
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan > > scanConverter(new rsb::converter::ProtocolBufferConverter<rst::vision::LocatedLaserScan >());
  rsb::converter::converterRepository<std::string>()->registerConverter(scanConverter);


  	  		// ---------------- Informer ---------------------



  // Prepare RSB listener for incomming lidar scans
  rsb::ListenerPtr lidarListener = factory.createListener(lidarInScope);
  boost::shared_ptr<rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>>lidarQueue(new rsc::threading::SynchronizedQueue<boost::shared_ptr<rst::vision::LocatedLaserScan>>(1));
  lidarListener->addHandler(rsb::HandlerPtr(new rsb::util::QueuePushHandler<rst::vision::LocatedLaserScan>(lidarQueue)));

  rst::vision::LocatedLaserScan scan;
  while( true ){

    // Fetch a new scan and store it to scan
    scan = *lidarQueue->pop();
    
  }

  return 0;
}

