
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <types/twbTracking.pb.h>

#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>


using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace rsb::converter;


#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For program options
#include <boost/program_options.hpp>

static std::string g_sServerScope = "/server";
static std::string g_sServerRequestName = "map";

int main(int argc, char **argv) {  

    namespace po = boost::program_options;

    po::options_description options("Allowed options");
    options.add_options()("help,h", "Display a help message.")
            ("serverScope,s", po::value < std::string > (&g_sServerScope),"Scope sending the server request")
            ("serverRequestName,i", po::value < std::string > (&g_sServerRequestName),"Server request name");

    // allow to give the value as a positional argument
    po::positional_options_description p;
    p.add("value", 1);

    po::variables_map vm;
    po::store(
            po::command_line_parser(argc, argv).options(options).positional(p).run(),
            vm);

    // first, process the help option
    if (vm.count("help")) {
        std::cout << options << "\n";
        exit(1);
    }
    
    // afterwards, let program options handle argument errors
    po::notify(vm);
    
    INFO_MSG( "Scope: " << g_sServerScope)
    INFO_MSG( "Request: " << g_sServerRequestName)


  ////////////////////////////////////////////////////////////////////////////////////////////////////
  // Register our converter within the collection of converters for
  // the string wire-type (which is used for arrays of octets in
  // C++).
  boost::shared_ptr<rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList> > pose2DListConverter(new rsb::converter::ProtocolBufferConverter<twbTracking::proto::Pose2DList>());
  rsb::converter::converterRepository<std::string>()->registerConverter(pose2DListConverter);


  rsb::Factory &factory = rsb::Factory::getInstance();

  // Create a client
  rsb::patterns::RemoteServerPtr remoteServer = factory.createRemoteServer(g_sServerScope);

  // Pop the images and show them
//  INFO_MSG( "Press ESC for exit, or any key for an request")
  INFO_MSG( "Send request")
  boost::shared_ptr<bool> request(new bool(false));
  boost::shared_ptr<twbTracking::proto::Pose2DList> result = remoteServer->call<twbTracking::proto::Pose2DList>(g_sServerRequestName, request, 25);


  return 0;

}
