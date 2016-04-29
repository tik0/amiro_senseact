
//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send target positions relative to the current
//               AMiRos local position with respect to its
//               coordinate system
//============================================================================

#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/filter/OriginFilter.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Version.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// RST
#include <rsb/converter/ProtocolBufferConverter.h>
#include <types/TargetPoseEuler.pb.h>
#include <rst/geometry/Translation.pb.h>

#include <ControllerAreaNetwork.h>

using namespace std;

void sendTargetPosition(rsb::EventPtr event, ControllerAreaNetwork &CAN) {
  
  // Get the message
  boost::shared_ptr<rst::geometry::TargetPoseEuler > message = boost::static_pointer_cast<rst::geometry::TargetPoseEuler >(event->getData());

  const ::rst::geometry::RotationEuler& rotation = message->target_pose().rotation();
  const ::rst::geometry::Translation& translation = message->target_pose().translation();

  types::position robotPosition;

  robotPosition.x = static_cast<int>(translation.x() * 1e6);
  robotPosition.y = static_cast<int>(translation.y() * 1e6);
  robotPosition.z = static_cast<int>(translation.z() * 1e6);
  robotPosition.f_x = static_cast<int>(rotation.roll() * 1e6);
  robotPosition.f_y = static_cast<int>(rotation.pitch() * 1e6);
  robotPosition.f_z = static_cast<int>(rotation.yaw() * 1e6);

  INFO_MSG("x: " << robotPosition.x << "um");
  INFO_MSG("f_z: " << robotPosition.f_z << "urad");
  INFO_MSG("time: " << static_cast<uint16_t>(message->target_time()) << "ms");
  // Set the target position
  CAN.setTargetPosition(robotPosition, static_cast<uint16_t>(message->target_time()));
}

static std::string rsbInScope = "/targetPositions";

int main (int argc, const char **argv){
 
  // Handle program options
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("inscope,i", po::value < std::string > (&rsbInScope), "Scope for receiving target positions");

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

  // Create the CAN interface
  ControllerAreaNetwork CAN;
  
  // Get the RSB factory
#if RSB_VERSION_NUMERIC<1200
  rsb::Factory& factory = rsb::Factory::getInstance();
#else
  rsb::Factory& factory = rsb::getFactory();
#endif

    // Register new converter
  boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::geometry::TargetPoseEuler> >
      converter(new rsb::converter::ProtocolBufferConverter<rst::geometry::TargetPoseEuler>());
  rsb::converter::converterRepository<std::string>()->registerConverter(converter);

  // Prepare RSB reader
  rsb::ReaderPtr reader = factory.createReader(rsbInScope);

  for(;;) {
    // Wait for the message
    sendTargetPosition(reader->read(), CAN);
  }

  return EXIT_SUCCESS;
}