//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Send integer vector with arbitrary values
//============================================================================




#define INFO_MSG_
// #define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
// #define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>


#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// Include own converter
#include <converter/vecIntConverter/main.hpp>

using namespace std;
using namespace muroxConverter;
using namespace boost;

int main (int argc, const char **argv){

      // Register new converter for std::vector<int>
      boost::shared_ptr<vecIntConverter> converter(new vecIntConverter());
      converterRepository<std::string>()->registerConverter(converter);
      
      // Prepare RSB informer
      rsb::Factory& factory = rsb::Factory::getInstance();
      rsb::Informer< std::vector<int> >::Ptr informer_vec = factory.createInformer< std::vector<int> > ("/IR");

      // Share the pointer to the IR data
      boost::shared_ptr< std::vector<int> > vecData(new std::vector<int> (12,0));
      
      for(;;) {
	// Get random values
	for (unsigned int idx = 0; idx < vecData->size(); idx++)
	  vecData->at(idx) = rand();
	// Send IR data
	informer_vec->publish(vecData);
	INFO_MSG( "Send vector<int> with 12 values to the scope \\IR" )
	// Sleep for 125 ms
	boost::this_thread::sleep( boost::posix_time::milliseconds(125) );
      }

      return EXIT_SUCCESS;
}