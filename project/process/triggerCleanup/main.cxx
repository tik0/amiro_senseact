//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the IR sensor data from a scope, then processes the data
//               and send a cleanup command, if something is right in front of the 
//               robot
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
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>
      
using namespace std;
using namespace muroxConverter;

// The IR-Data ticks, where the heading needs to stop
int g_iIRDataUpperLimit = 1000;
unsigned int g_uiIROverUpperLimitCounter = 0;
unsigned int g_uiIROverUpperLimitThreashold = 8;

std::string g_sInScope_IRData = "/IR";
std::string g_sOutScope_RemoteCleanup = "/tabletop/cleanup";
std::string g_sRemoteSocketServer = "192.168.2.199";

void fusion(rsb::EventPtr event, rsb::Informer< void >::Ptr informerRemoteCleanup) {

      // Get the message
      shared_ptr<std::vector<int> > message = static_pointer_cast<std::vector<int> >(event->getData());

      int iIR00 = message->at(0);
      int iIR11 = message->at(1);
      
      // If the IR-Data is high enough increment the counter, ...
      if (iIR00 > g_iIRDataUpperLimit || iIR11 > g_iIRDataUpperLimit) {
	++g_uiIROverUpperLimitCounter;
	// If the counter is high enough, send the cleanup command
	if ( g_uiIROverUpperLimitCounter >= g_uiIROverUpperLimitThreashold ) {
	  // Just a void for sending nothing
	  boost::shared_ptr< void > publishVoid(new int(0));
	  informerRemoteCleanup->publish(publishVoid);
#ifdef __arm__
	  ::system("/home/root/leds_green_brightness.sh 512");
#endif
	}

      } else {  // ... otherwise reset it
	g_uiIROverUpperLimitCounter = 0;
#ifdef __arm__
	::system("/home/root/leds_brightness.sh 0");
#endif
      }
}

int main (int argc, const char **argv){

   namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
	    ("inscopeIRData,ir", po::value < std::string > (&g_sInScope_IRData), "Scope for receiving IR data.")
	    ("remoteSocketServer,s", po::value < std::string > (&g_sRemoteSocketServer), "IP of remote socket server.")
            ("outscopeRemoteCleanup,orc", po::value < std::string > (&g_sOutScope_RemoteCleanup),"Scope for sending cleanup command.");
	    
    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).positional(p).run(),vm);

    // first, process the help option
    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }

    // afterwards, let program options handle argument errors
    po::notify(vm);
    
    // Get the RSB factory
    rsb::Factory& factory = rsb::Factory::getInstance();
    
    // Register new converter for std::vector<int>
    shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
    converterRepository<std::string>()->registerConverter(converterVecInt);
	  
    // Prepare RSB reader
    ReaderPtr reader = factory.createReader(g_sInScope_IRData);

    
	  //////////////////// CREATE A CONFIG TO COMMUNICATE WITH ANOTHER SERVER ////////
    ///////////////////////////////////////////////////////////////////////////////
        // Get the global participant config as a template
        rsb::ParticipantConfig tmpPartConf = factory.getDefaultParticipantConfig();
              {
                // Get the options for socket transport, because we want to change them
                rsc::runtime::Properties tmpPropSocket  = tmpPartConf.mutableTransport("socket").getOptions();

                // enable socket transport
                std::string enabled = "0";
                tmpPropSocket["enabled"] = boost::any(enabled);

                // this node is the server, all other nodes clients server = 0 !!!
                std::string server = "0";
                tmpPropSocket["server"] = boost::any(server);

                // Change the config
                // ToDo Change the host for the real server
  //         std::string host = "192.168.0.200";
                tmpPropSocket["host"] = boost::any(g_sRemoteSocketServer);

                // Write the socket tranport properties back to the participant config
                tmpPartConf.mutableTransport("socket").setOptions(tmpPropSocket);
              }
              {
                // Get the options for spread transport, because we want to change them
                rsc::runtime::Properties tmpPropSpread = tmpPartConf.mutableTransport("spread").getOptions();

                std:: cout << tmpPropSpread << std::endl;

                // enable socket transport
                std::string enabled = "1";
                tmpPropSpread["enabled"] = boost::any(enabled);

                // the port of the server
                std::string port = "4816";
                tmpPropSpread["port"] = boost::any(port);

                // Change the config
                // ToDo Change the host for the real server
  //            std::string host = "192.168.0.200";
                tmpPropSpread["host"] = boost::any(g_sRemoteSocketServer);

                std:: cout << tmpPropSpread << std::endl;

                // Write the socket tranport properties back to the participant config
                tmpPartConf.mutableTransport("spread").setOptions(tmpPropSpread);
              }
	 ///////////////////////////////////////////////////////////////////////////////
	 ///////////////////////////////////////////////////////////////////////////////
    
    
    // Prepare remote informer
    rsb::Informer< void >::Ptr informerRemoteCleanup;

    try {
      informerRemoteCleanup = factory.createInformer< void > (g_sOutScope_RemoteCleanup,tmpPartConf);
#ifdef __arm__
      ::system("/home/root/leds_trigger_none.sh");
      ::system("/home/root/leds_brightness.sh 0");
      ::system("/home/root/leds_green_brightness.sh 512");
      boost::this_thread::sleep(boost::posix_time::seconds(1));
      ::system("/home/root/leds_green_brightness.sh 512");
      ::system("/home/root/leds_brightness.sh 0");
#endif
    }
    catch(std::exception& e) {
      ERROR_MSG( "Remote connection not established" );
#ifdef __arm__
      ::system("/home/root/leds_trigger_none.sh");
      ::system("/home/root/leds_brightness.sh 0");
      ::system("/home/root/leds_red_brightness.sh 512");
#endif
      return -1;
    }

    // Print events as they are received.
    while (true) {
        fusion(reader->read(), informerRemoteCleanup);
    }

      return EXIT_SUCCESS;
}
