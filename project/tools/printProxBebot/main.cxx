//============================================================================
// Name        : main.cxx
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Reads the IR sensor data
//============================================================================




#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include "infraRed.h"
#include <iostream>

#include "fFlags.hpp"
#include <boost/thread.hpp> // this_thread and posix_time

using namespace std;

int main (int argc, const char **argv){

	INFO_MSG( "This program reads the IR values. Use -h flag for infos." )
	
	fFlags::handleCommandline(argc, argv);
	
	INFO_MSG( "Period T= " << fFlags::uiPeriod_ms << " ms")

	InfraRed myIR;

	for(;;) {
                // Read the IR data
		myIR.getIRData();
		// Print the IR data
	        myIR.printIRData();
		// Sleep for 
		boost::this_thread::sleep( boost::posix_time::milliseconds(fFlags::uiPeriod_ms) );
	}

	return EXIT_SUCCESS;
}