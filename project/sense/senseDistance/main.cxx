//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the distance information of DS_ODSL96B_M_TOF_S12
//               and send it via RSB
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>


// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// RST
#include <rsb/converter/ProtocolBufferConverter.h>

// Proto types
#include <types/LeuzODS9ODS96B.pb.h>

// Serial interface
#include "rs232.h"

#include <stdint.h>

#define BUFSIZE 3

using namespace std;

static std::string rsbOutScope = "/distance";
static uint32_t rsbPeriod = 0;
static int32_t cport_nr = 0;
static int32_t bdrate = 19200;

int main(int argc, char **argv) {
  INFO_MSG("")
  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending odometry values")
    ("period,t", po::value < uint32_t > (&rsbPeriod), "Update interval (0 for maximum rate)")
    ("port,p", po::value < int32_t > (&cport_nr), "Portnumber of ttyS<0-15> (Standardvalue: 0)")
    ("bdrate,b", po::value < int32_t > (&bdrate), "Baudrate (Standardvalue: 19200)");

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

  // Register new converter for std::vector<int>
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::LeuzODS9ODS96B> >
      converter(new rsb::converter::ProtocolBufferConverter<rst::claas::LeuzODS9ODS96B>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB informer
  rsb::Factory& factory = rsb::Factory::getInstance();
  rsb::Informer< rst::claas::LeuzODS9ODS96B >::Ptr informer = factory.createInformer< rst::claas::LeuzODS9ODS96B > (rsbOutScope);


  // Datastructure for the messages
  rsb::Informer<rst::claas::LeuzODS9ODS96B>::DataPtr msg(new rst::claas::LeuzODS9ODS96B);

  
  // Set up the serial connection
  int n;
  unsigned char buf[BUFSIZE];
  char mode[]={'8','N','1',0};



  // Start the synchronized communication
  if(RS232_OpenComport(cport_nr, bdrate, mode, BUFSIZE))
  {
    ERROR_MSG("Can not open comport")
    return(1);
  }

  // Raw distances
  uint16_t distance, distance1, distance2, distance3;

  while(1) {
    n = RS232_PollComport(cport_nr, buf, BUFSIZE);
    INFO_MSG("Bytes: " << n)
    
    if ((buf[0] & 0x03) != 0x00) {
      // Syncing to the last byte
      if(RS232_OpenComport(cport_nr, bdrate, mode, 1))
      {
        ERROR_MSG("Can not open comport")
        return(1);
      }

      while(1) {
        n = RS232_PollComport(cport_nr, buf, 1);
        if (n == 1) {
          if ((buf[0] & 0x03) == 0x02) {
            ERROR_MSG("Sync")
            break;
          }
        }
      }

      // Start the synchronized communication
      if(RS232_OpenComport(cport_nr, bdrate, mode, BUFSIZE))
      {
        ERROR_MSG("Can not open comport")
        return(1);
      }
    }

    // We received something
    if (n == 3)
    {

      // Print the incomming byteorder
      DEBUG_MSG( "0 " << (buf[0] & 3))
      DEBUG_MSG( "1 " << (buf[1] & 3))
      DEBUG_MSG( "2 " << (buf[2] & 3))
      DEBUG_MSG( "NumberBytes " << n)

      // Decoding of the distance values
      for(uint8_t idx = 0; idx < 3; ++idx) {
        uint16_t tmp = buf[idx];
        switch (tmp & 0x0003) {
          case 0: distance1 = (tmp >> 2); break;
          case 1: distance2 = ((tmp >> 2) << 6); break;
          case 2: distance3 = ((tmp >> 2) << 12); break;
          default: break;
        }
      }
      distance = distance1 + distance2 + distance3;

      // Convert it to rsb data
      msg->set_distance_mm(static_cast<uint32_t>(distance));
      msg->set_distance_in_range(false);
      msg->set_error_signal_too_week(false);
      msg->set_device_error_signal(false);
      msg->set_device_error_laser(false);
      msg->set_distance_out_of_range(false); 

    // Aufteilung des Rohsignals in die Daten
    // Error: Signal zu schwach, Gerätefehler Signal und Gerätefehler Laser
    // Distance in range: Distanzwert linear
    // Distance out of range: Distanzwert linearität undefiniert
      if(distance < 10501)
        msg->set_distance_in_range(true);
      else if(distance == 65535)
        msg->set_error_signal_too_week(true);
      else if(distance == 65534)
        msg->set_device_error_signal(true);
      else if(distance == 65533)
        msg->set_device_error_laser(true);
      else 
        msg->set_distance_out_of_range(true);

      INFO_MSG( "Distance " << distance )

      informer->publish(msg);
    }

    // Sleep for a while
    boost::this_thread::sleep( boost::posix_time::milliseconds(rsbPeriod) );
  }

  return EXIT_SUCCESS;
}
