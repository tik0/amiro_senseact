//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Starts a dummy socket server
//============================================================================

#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <iostream>

#include <rsb/Factory.h>
#include <rsb/Event.h>

using namespace std;

void output(rsb::EventPtr event) {
    std::cout << "Received event" << std::endl;
}

int main (int argc, const char **argv){

	INFO_MSG( "This program does nothing but a server." )

        // Get the RSB factory
        rsb::Factory& factory = rsb::Factory::getInstance();
	// Prepare RSB reader (listen to the root scope)
        rsb::ReaderPtr reader = factory.createReader("/xxx/xxx/xxx");
	for(;;) {
                output(reader->read());
	}

	return EXIT_SUCCESS;
}
