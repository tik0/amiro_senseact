
#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include "../../includes/MSG.h"

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

// RSB
#include <rsb/filter/OriginFilter.h>
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
#include <types/SickLdMRS4002.pb.h>

#include <kbhit.hpp>

using namespace boost;
using namespace std;
using namespace rsb;
using namespace rsb::converter;

int main(int argc, const char **argv){

  // Handle program options
  namespace po = boost::program_options;
  
  std::string outScope = "/lidar";

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&outScope), "Scope for sending the dummy data");

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
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::SickLdMRS400102> > converter(new rsb::converter::ProtocolBufferConverter<rst::claas::SickLdMRS400102>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB informer
  rsb::Informer< rst::claas::SickLdMRS400102 >::Ptr informer_vec = factory.createInformer< rst::claas::SickLdMRS400102 > (outScope);

  // Define the steering message for sending over RSB
  boost::shared_ptr< rst::claas::SickLdMRS400102 > scanData(new rst::claas::SickLdMRS400102);
  scanData->set_magicword(0xAFFEC0C2);
  scanData->set_sizemsg(100);  // Dummy value
  scanData->set_deviceid(1);
  scanData->set_datatype(2);
  scanData->set_timemsgsent(3);

  // Scan data
  scanData->set_measurenumber(4);
  scanData->set_sensorstatus(5);
  scanData->set_synchrophase(6);
  scanData->set_timestartmeasure(7);
  scanData->set_timeendmeasure(8);
  scanData->set_angularstepsperrotation(9);
  scanData->set_startangle(10);
  scanData->set_endangle(11);
  scanData->set_numbermeasuredpoints(12);

  // Fill the raw data with dummy data
  ::google::protobuf::RepeatedField< ::google::protobuf::uint64 > *rawData = scanData->mutable_measuredpoints();
  for (size_t idx = 0; idx < 1000; ++idx)
    rawData->Add(idx);

  uint8_t KB_code = 0;
  while(KB_code != KB_ESCAPE )
  { 
    if (kbhit())
    {
      KB_code = getchar();
      INFO_MSG( "Press ESC to exit" )
      INFO_MSG( "KB_code = " << KB_code )
    }
      informer_vec->publish(scanData);
      INFO_MSG( " Data sent" )
    // Sleep for 0.5 second
    usleep(500000);
  }

  return 0;

}
