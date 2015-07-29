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

class IbeoScalaDataListener : public ibeosdk::DataListener<ibeosdk::Scan2208>

{

public:
	IbeoScalaDataListener(std::string ip, std::string scalaScope);
	virtual ~IbeoScalaDataListener();

	//void onData(const ibeosdk::FrameEndSeparator* const fes);
	void onData(const ibeosdk::Scan2208* const scan);

private:
	std::string scalaIp;

	rsb::Informer<rst::claas::IbeoScala_1403>::DataPtr msgIbeoScala;
	rsb::Informer< rst::claas::IbeoScala_1403 >::Ptr informer;
};

#endif /* SRC_IBEOSCALADATALISTENER_HPP_ */
