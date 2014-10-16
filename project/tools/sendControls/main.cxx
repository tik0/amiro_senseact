
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

// Reading inScope and outScope
#include <oFlags.hpp>

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




using namespace boost;
using namespace std;
//using namespace cv;
using namespace rsb;
using namespace muroxConverter;
using namespace rsb::converter;
 
// For checking character pressed in the console
#include <kbhit.hpp>


int main(int argc, const char **argv){
  
      INFO_MSG( "Before reading values " )
      
      INFO_MSG( "Outscope: " << oFlags::outScope)
      
      oFlags::handleCommandline(argc, argv);
      INFO_MSG( "After reading values " )
      
      INFO_MSG( "Outscope: " << oFlags::outScope)
      
      
    // Get the RSB factory
    rsb::Factory& factory = rsb::Factory::getInstance();
    
    // Register new converter for std::vector<int>
    shared_ptr<vecIntConverter> converterVecInt(new vecIntConverter());
    converterRepository<std::string>()->registerConverter(converterVecInt);

    // Prepare RSB informer
    rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > (oFlags::outScope);

    // Calculate the new steering (two elements with 0 initialized)
    boost::shared_ptr< std::vector<int> > vecSteering(new std::vector<int> (2,0));

   int KB_code=0;

   while(KB_code != KB_ESCAPE )
   { 
     if (kbhit())
      {
            KB_code = getchar();
	    INFO_MSG( "KB_code = " << KB_code )

            switch (KB_code)
            {
                case KB_A:
			   // Degree per second
                           vecSteering->at(1) = vecSteering->at(1) + 10;
			   informer_vec->publish(vecSteering);
                break;

                case KB_D:
		           // Degree per second
                           vecSteering->at(1) = vecSteering->at(1) - 10;
			   informer_vec->publish(vecSteering);
                break;

                case KB_W:
		           // Milimeter per second
                           vecSteering->at(0) = vecSteering->at(0) + 10;
			   informer_vec->publish(vecSteering);
                break;

                case KB_S:
			   // Milimeter per second
                           vecSteering->at(0) = vecSteering->at(0) - 10;
			   informer_vec->publish(vecSteering);
                break;
		case KB_SPACE:
			   vecSteering->at(0) = 0;
			   vecSteering->at(1) = 0;
			   informer_vec->publish(vecSteering);
                break;
		case KB_ENTER:
			   informer_vec->publish(vecSteering);
                break;

            }        
            INFO_MSG( " v = " << vecSteering->at(0) << ", w = " << vecSteering->at(1) )

      }
  }

  return 0;

}
