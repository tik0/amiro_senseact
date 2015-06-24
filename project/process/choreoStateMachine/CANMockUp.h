/*
 * CANMockUp.h
 *
 *  Created on: Mar 11, 2015
 *      Author: florian
 */

#ifndef CANMOCKUP_H_
#define CANMOCKUP_H_

#include <iostream>
#include <Color.h>  // Color types
using namespace std;

// CAN MockUp, just print received values
class CANMockUp {
public:
	CANMockUp(){};
	virtual ~CANMockUp(){};

	void setTargetSpeed(int v, int w) {
		printf("v: %d w: %d\n",v,w);
	}

	void setLightBrightness(uint8_t brightness) {
		printf("brightness: %d\n",brightness);
	}

	void setLightColor(int index, amiro::Color color) {
		printf("led%d: [%d,%d,%d]\n",index,color.getBlue(),color.getGreen(),color.getRed());
	}
};

#endif /* CANMOCKUP_H_ */
