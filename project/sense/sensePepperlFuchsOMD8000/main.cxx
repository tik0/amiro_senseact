//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the distance and echo information of Pepperl+Fuchs OMD8000
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
#include <types/PepperlFuchsOMD8000.pb.h>

// Serial interface
#include "rs232.h"

#include <stdint.h>
#include <chrono>
#include <thread>
#include <functional>

#define BUFSIZE 50
#define NUMDISTANCES 11
#define REQ_BUFSIZE 5
static unsigned char reqString[REQ_BUFSIZE] = {0xde, 0x01, 0x05, 0x59, 0x83};

using namespace std;

static std::string rsbOutScope = "/sense/PuFOMD8000/1";
static uint32_t rsbPeriod = 20;
static int32_t cport_nr = 0;
static int32_t bdrate = 115200;

void timer_start(std::function<void(void)> func, uint32_t interval)
{
    std::thread([func, interval]() {
        while (true)
        {
            func();
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
    }).detach();
}


void sensorRequest()
{
  RS232_SendBuf(cport_nr, reqString, REQ_BUFSIZE);
}

int main(int argc, char **argv) {
  std::vector<double> coordinates{1312, -553, 1442,004, 0, 90, 0};
  INFO_MSG("")
  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending odometry values")
    ("t_period,t", po::value < uint32_t > (&rsbPeriod), "Update interval in ms ( 20 for maximum rate)")
    ("port,p", po::value < int32_t > (&cport_nr), "Portnumber of ttyS<0-15> (Standardvalue: 0)")
    ("bdrate,b", po::value < int32_t > (&bdrate), "Baudrate (Standardvalue: 115200)")
	("q_position,q", po::value <std::vector<double> > (&coordinates)->multitoken(), "Sensor Poistion: x y z alpha beta gamma, default = 0, 0, 0, 0, 0, 0");

  // allow to give the value as a positional argument
  //po::positional_options_description p;
  //p.add("value", 1);

  po::variables_map vm;
  po::store( po::command_line_parser(argc, argv).options(options).style(po::command_line_style::unix_style ^ po::command_line_style::allow_short).run(), vm);

  // first, process the help option
  if (vm.count("help")) {
      std::cout << options << "\n";
      exit(1);
  }

  // afterwards, let program options handle argument errors
  po::notify(vm);

  // Register new converter for std::vector<int>
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::PepperlFuchsOMD8000> >
      converter(new rsb::converter::ProtocolBufferConverter<rst::claas::PepperlFuchsOMD8000>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB informer
  rsb::Factory& factory = rsb::getFactory();
  rsb::Informer< rst::claas::PepperlFuchsOMD8000 >::Ptr informer = factory.createInformer< rst::claas::PepperlFuchsOMD8000 > (rsbOutScope);


  // Datastructure for the messages
  rsb::Informer<rst::claas::PepperlFuchsOMD8000>::DataPtr msg(new rst::claas::PepperlFuchsOMD8000);

  // Set position values for the sensor
  msg->mutable_pose()->mutable_translation()->set_x(coordinates.at(0));
  msg->mutable_pose()->mutable_translation()->set_y(coordinates.at(1));
  msg->mutable_pose()->mutable_translation()->set_z(coordinates.at(2));
  msg->mutable_pose()->mutable_rotation()->set_roll(coordinates.at(3));
  msg->mutable_pose()->mutable_rotation()->set_pitch(coordinates.at(4));
  msg->mutable_pose()->mutable_rotation()->set_yaw(coordinates.at(5));
  
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

  // Start requsting
  timer_start(sensorRequest, rsbPeriod);

  // Raw distances
  uint16_t distances[NUMDISTANCES];
  uint16_t echos[NUMDISTANCES];
  uint16_t noTarget[NUMDISTANCES];

  while(1) {
    n = RS232_PollComport(cport_nr, buf, BUFSIZE);
    INFO_MSG("Bytes: " << n)

    // We received something
    if (n == BUFSIZE) {

      // Decoding of the distance and echo values
      uint8_t idxCounter = 0;
      for(uint8_t idx = 4; idx < 48; idx=idx+4) {
        distances[idxCounter] = buf[idx+1];
        echos[idxCounter]     = buf[idx+3];
        distances[idxCounter] <<=8;
        echos[idxCounter]     <<=8;
        distances[idxCounter] = distances[idxCounter] | buf[idx];
        echos[idxCounter]     = echos[idxCounter]     | buf[idx+2];
        if (distances[idxCounter] == 0xFFFF) {
          noTarget[idxCounter] = true;
        } else {
          noTarget[idxCounter] = false;
        }
        ++idxCounter;
      }

      // Convert it to rsb data
      msg->Clear();  // Delete the content
      for(uint8_t idxCounter = 0; idxCounter < NUMDISTANCES; ++idxCounter) {
        msg->mutable_distance_mm()->Add(distances[idxCounter]);
        msg->mutable_echo()->Add(echos[idxCounter]);
        msg->mutable_notarget()->Add(noTarget[idxCounter]);
      }

      informer->publish(msg);
    }
  }

  return EXIT_SUCCESS;
}
