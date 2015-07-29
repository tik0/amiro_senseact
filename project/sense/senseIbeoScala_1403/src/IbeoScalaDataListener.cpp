/*
 * IbeoScalaDataListener.cpp
 *
 *  Created on: 29.07.2015
 *      Author: itsowl
 */

#include <src/IbeoScalaDataListener.hpp>
#include <iostream>

IbeoScalaDataListener::IbeoScalaDataListener(std::string ip, std::string scalaScope) : scalaIp(ip), msgIbeoScala(new rst::claas::IbeoScala_1403){

	rsb::Factory& factory = rsb::getFactory();
	informer = factory.createInformer< rst::claas::IbeoScala_1403 > (scalaScope);

}

IbeoScalaDataListener::~IbeoScalaDataListener() {

}

void IbeoScalaDataListener::onData(const ibeosdk::Scan2208* const scan){
	msgIbeoScala->set_uniqueidentifier(scalaIp);
	msgIbeoScala->set_scannumber(scan->getScanNumber());
	msgIbeoScala->set_scannertype(scan->getScannerType());
	msgIbeoScala->set_scannerstatus(scan->getScannerStatus());
	msgIbeoScala->set_angleticksrotation(scan->getAngleTicksPerRotation());

	msgIbeoScala->set_mountingyaw(scan->getMountingPosition().getYawAngle());
	msgIbeoScala->set_mountingpitch(scan->getMountingPosition().getPitchAngle());
	msgIbeoScala->set_mountingroll(scan->getMountingPosition().getRollAngle());
	msgIbeoScala->set_mounting_x(scan->getMountingPosition().getX());
	msgIbeoScala->set_mounting_y(scan->getMountingPosition().getY());
	msgIbeoScala->set_mounting_z(scan->getMountingPosition().getZ());
	msgIbeoScala->set_deviceid(scan->getDeviceId());

	msgIbeoScala->set_scanstarttime(scan->getSubScans().at(0).getStartScanTimestamp());
	msgIbeoScala->set_scanendtime(scan->getSubScans().at(0).getEndScanTimestamp());
	msgIbeoScala->set_startangle(scan->getSubScans().at(0).getStartScanAngle());

	msgIbeoScala->set_mirrorside(scan->getSubScans().at(0).getMirrorSide());

	msgIbeoScala->set_mirrortilt(scan->getSubScans().at(0).getMirrorTilt());
	msgIbeoScala->set_numberscanpoints(scan->getSubScans().at(0).getNbOfPoints());

	for (auto &pointVector : scan->getSubScans().at(0).getScanPoints()){
		msgIbeoScala->add_echo(pointVector.getEchoId());
		msgIbeoScala->add_layer(pointVector.getLayerId());
		msgIbeoScala->add_scanpointflags(pointVector.getFlags());
		msgIbeoScala->add_horizontalangle(pointVector.getHorizontalAngle());
		msgIbeoScala->add_radialdistance(pointVector.getRadialDistance());
		msgIbeoScala->add_echopulsewidth(pointVector.getEchoPulseWidth());
	}

	informer->publish(msgIbeoScala);
	msgIbeoScala->Clear();
}
