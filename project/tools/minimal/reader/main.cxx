
#include <iostream>
#include <boost/thread.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb0.9/rsb/filter/OriginFilter.h>
#include <rsb0.9/rsb/Informer.h>
#include <rsb0.9/rsb/Factory.h>
#include <rsb0.9/rsb/Event.h>
#include <rsb0.9/rsb/Handler.h>
#include <rsb0.9/rsb/converter/Repository.h>

// #include <rsb/filter/OriginFilter.h>
// #include <rsb/Informer.h>
// #include <rsb/Factory.h>
// #include <rsb/Event.h>
// #include <rsb/Handler.h>
// #include <rsb/converter/Repository.h>


using namespace std;
void process(rsb::EventPtr event) {
  std::cout << "Got Message" << std::endl;
}


int main (int argc, const char **argv){
  
	
	// Get the RSB factory
	rsb::Factory& factory = rsb::Factory::getInstance();

	// Prepare RSB reader
	rsb::ReaderPtr reader = factory.createReader("/scope");


	for(;;) {
          	// Read the cmd and send it to the motor
                process(reader->read());
	}

	return EXIT_SUCCESS;
}