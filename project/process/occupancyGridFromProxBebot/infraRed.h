/*
 * infraRed.h
 *
 *  Created on: 07.03.2014
 *      Author: tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
 */

#ifndef INFRARED_H_
#define INFRARED_H_

// Debug messages
// These are pre-processor flags which can be commented out
// to disable the messages
#define INFO_MSG_
#define DEBUG_MSG_
#define SUCCESS_MSG_
#define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <bebot.h>
#include <vector>
#include <string>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
// #include "rsbInformer.h"

class InfraRed{

public:

	InfraRed();
	~InfraRed();

	/**
	 * Gets the infrared data from bebot and saves it to the vector irSensors
	 */
	void getIRData();

	/**
	 * Prints out the vector irSensors
	 */
 	void printIRData();

	/**
	 * Stores the IR data
	 */
	boost::shared_ptr< std::vector<int> > irSensors;
private:
	
	/**
	 * Holds the bebot structure
	 */
	struct bebot BEBOT;
	
	/**
	* Maximum potence of the IR values.
	* e.g. If max. IR value is 3854 -> uiMaxPotIR = 4
	* e.g. If max. IR value is   54 -> uiMaxPotIR = 2
	*/ 
	static const std::size_t uiMaxPotIR = 4;

};
#endif /* INFRARED_H_ */