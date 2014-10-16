/*
 * rsbInformer.cpp
 *
 *  Created on: Nov 26, 2013
 *      Author: Kai
 */

#include "rsbInformer.h"

namespace SensorInformer {

SensorInformer::rsbInformer::rsbInformer() {
	this->isSetUp = false;
}

SensorInformer::rsbInformer::~rsbInformer() {

}

int SensorInformer::rsbInformer::setup_int(std::string scope)  {
	if (this->isSetUp) {
		std::cout << "[rsbInformer] Error - RSB is already set up" << std::endl;
		return EXIT_FAILURE;
	}
	rsb::Factory& factory = rsb::Factory::getInstance();
	try {
		informer_int = factory.createInformer<int> (scope);
	}
	catch(std::exception& e) {
		std::cout << "[rsbInformer] Error - Sensor-Informer could not be created" << std::endl;
		return EXIT_FAILURE;
	}
	this->isSetUp = true;
	return EXIT_SUCCESS;
}

int SensorInformer::rsbInformer::setup_vec(std::string scope)  {
	if (this->isSetUp) {
		std::cout << "[rsbInformer] Error - RSB is already set up" << std::endl;
		return EXIT_FAILURE;
	}
	rsb::Factory& factory = rsb::Factory::getInstance();

	try {
		informer_vec = factory.createInformer<std::vector<int> > (scope);
	}
	catch(std::exception& e) {
		std::cout << "[rsbInformer] Error - Sensor-Informer could not be created" << std::endl;
		return EXIT_FAILURE;
	}
	this->isSetUp = true;
	return EXIT_SUCCESS;
}
/*
int SensorInformer::rsbInformer::setup_mat(std::string scope)  {
	if (this->isSetUp) {
		std::cout << "Error - RSB is already set up" << std::endl;
		return EXIT_FAILURE;
	}
	rsb::Factory& factory = rsb::Factory::getInstance();
	informer_mat = factory.createInformer<cv::Mat> (scope);
	this->isSetUp = true;
	return EXIT_SUCCESS;
}
*/


int SensorInformer::rsbInformer::publish(int value) {
	if (!this->isSetUp) {
		std::cout << "[rsbInformer] Error - Try to publish while RSB is not set up" << std::endl;
		return EXIT_FAILURE;
	}
	rsb::Informer<int>::DataPtr s(new int(value));
	this->informer_int->publish(s);
	return EXIT_SUCCESS;
}

int SensorInformer::rsbInformer::publish(std::vector<int> value) {
	if (!this->isSetUp) {
		std::cout << "[rsbInformer] Error - Try to publish while RSB is not set up" << std::endl;
		return EXIT_FAILURE;
	}
	rsb::Informer<std::vector<int> >::DataPtr s(new std::vector<int>(value));
	this->informer_vec->publish(s);
	return EXIT_SUCCESS;
}
/*
int SensorInformer::rsbInformer::publish(cv::Mat value) {
	if (!this->isSetUp) {
		std::cout << "Error - Try to publish while RSB is not set up" << std::endl;
		return EXIT_FAILURE;
	}
	rsb::Informer<cv::Mat>::DataPtr s(new cv::Mat(value));
	this->informer_mat->publish(s);
	return EXIT_SUCCESS;
}
*/
}


