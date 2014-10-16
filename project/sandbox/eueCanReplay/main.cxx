// Messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// For running system comands
#include <stdlib.h>

// For using kbhit
#include <unistd.h>

// For checking character pressed in the console
#include <kbhit.hpp>

// For using vector
#include <vector>

// For using string
#include <string>

// For program options
#include <boost/program_options.hpp>

// RSB
#include <boost/shared_ptr.hpp>
#include <rsb/Factory.h>
#include <rsb/converter/Repository.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/filter/OriginFilter.h>
#include <rsc/threading/SynchronizedQueue.h>
#include <rsb/QueuePushHandler.h>

#include <chrono>
#include <thread>

#include <iostream>
#include <fstream>

using namespace boost;
using namespace std;
using std::vector;
using namespace rsb;
using namespace rsb::converter;

std::string logFile = "./LoggerExample.asc";
std::string outScope = "/log";


void usecond_wait( unsigned int us )
{
    std::chrono::microseconds dura( us );
    std::this_thread::sleep_for( dura );
}

int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description options("Allowed options");
  options.add_options()("help,h", "Display a help message.")
    ("file,f", po::value < std::string > (&logFile), "Filename of logfile")
    ("scope,s", po::value < std::string > (&outScope), "Scope");

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

  
      /////////////////// Create Scope ///////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////

  // Create the factory
//     rsb::Factory &factory = rsb::Factory::getInstance();
    Factory& factory = getFactory();
    
    // Informer for the steering commands
    rsb::Informer< std::string >::Ptr informer = factory.createInformer< std::string > (outScope);

  ifstream myfile;
  myfile.open (logFile.c_str());
  char a;
  string startStr("   0.000000    EnvTcpServerPort := 2ee2");
  string line;
  int idx = 0, jmpOver = 11;
//   boost::shared_ptr< string > sendStr;
   Informer<string>::DataPtr sendStr(new string("example payload"));
  if (myfile.is_open()) {
    while (std::getline(myfile, line))
    {
        if (++idx > jmpOver) {
          *sendStr = line;
          cout << line << endl;
          cout << strcmp(startStr.c_str(),line.c_str());
          std::string token(line,0,12);
          cout << "\n token: "<< token <<" " << atof(token.c_str()) <<  endl;
          usecond_wait((unsigned int)(atof(token.c_str()) * 1e6));
//           cin >> a;
          informer->publish(sendStr);
        }
    }
  } else {
    cout << "Unable to open file";
  }
  
  
  
  myfile.close();

  return 0;
}