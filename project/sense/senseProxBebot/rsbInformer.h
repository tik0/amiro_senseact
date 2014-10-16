/*
 * rsbInformer.h
 *
 *  Created on: Nov 25, 2013
 *      Author: Kai
 */

#ifndef RSBINFORMER_H_
#define RSBINFORMER_H_

#include <stdlib.h>
#include <iostream>

#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

#include <rsb/QueuePushHandler.h>
#include <rsc/threading/SynchronizedQueue.h>

#include <rsb/transport/inprocess/Bus.h>
#include <rsb/transport/inprocess/InPullConnector.h>
#include <rsb/transport/inprocess/InPushConnector.h>
#include <rsb/transport/inprocess/InConnector.h>
#include <rsb/transport/inprocess/OutConnector.h>

//#include <rst/converters/opencv/IplImageConverter.h>

#include <boost/shared_ptr.hpp>

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>


namespace SensorInformer {

/**
 * This is an abstract class for RSB-publishing.
 */
class rsbInformer {

	public:
		rsbInformer();
		~rsbInformer();

		/**
		 * Sets up an RSB-Informer with declared type at Scope scope.
		 */
		int setup_int(std::string scope);
		int setup_vec(std::string scope);
	//	int setup_mat(std::string scope);

		/**
		 * Publishes a value to the declared scope.
		 */
		int publish(int value);
		int publish(std::vector<int> value);
	//	int publish(cv::Mat);

	private:
		rsb::Informer<int>::Ptr informer_int;
		rsb::Informer<std::vector<int> >::Ptr informer_vec;
//		rsb::Informer<cv::Mat>::Ptr informer_mat;

		bool isSetUp;
	};
}

#endif /* RSBINFORMER_H_ */
