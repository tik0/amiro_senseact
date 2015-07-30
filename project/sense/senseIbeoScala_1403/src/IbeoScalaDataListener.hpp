/*
 * IbeoScalaDataListener.hpp
 *
 *  Created on: 29.07.2015
 *      Author: itsowl
 */

#ifndef SRC_IBEOSCALADATALISTENER_HPP_
#define SRC_IBEOSCALADATALISTENER_HPP_

#include <ibeosdk/scala.hpp>
#include <ibeosdk/IpHelper.hpp>

#include <types/IbeoScala_1403.pb.h>
#include <string>

// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
#include <rsb/converter/Repository.h>

// RST
#include <rsb/converter/ProtocolBufferConverter.h>

class IbeoScalaDataListener : public ibeosdk::DataListener<ibeosdk::Scan2208>,
							  public ibeosdk::DataListener<ibeosdk::DeviceStatus>,
							  public ibeosdk::DataListener<ibeosdk::ObjectListScala2271>

{

public:
	IbeoScalaDataListener(std::string ip, std::string scalaScope);
	virtual ~IbeoScalaDataListener();

	void onData(const ibeosdk::Scan2208* const scan);
	void onData(const ibeosdk::DeviceStatus* const deviceStatus);
	void onData(const ibeosdk::ObjectListScala2271* const objs);

private:
	std::string scalaIp;

	rsb::Informer<rst::claas::IbeoScala_1403_Scan_2208>::DataPtr msgIbeoScala;
	rsb::Informer< rst::claas::IbeoScala_1403_Scan_2208 >::Ptr informer;

	rsb::Informer<rst::claas::IbeoScala_1403_ObjectData_2271>::DataPtr msgIbeoScalaObjectData_2271;
	rsb::Informer< rst::claas::IbeoScala_1403_ObjectData_2271 >::Ptr informer_ObjectData_2271;

	rsb::Informer<rst::claas::IbeoScala_1403_DeviceStatus>::DataPtr msgIbeoScalaDeviceStatus;
	rsb::Informer< rst::claas::IbeoScala_1403_DeviceStatus >::Ptr informer_DeviceStatus;
};

#endif /* SRC_IBEOSCALADATALISTENER_HPP_ */
