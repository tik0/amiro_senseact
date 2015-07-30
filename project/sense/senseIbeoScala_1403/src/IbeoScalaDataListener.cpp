/*
 * IbeoScalaDataListener.cpp
 *
 *  Created on: 29.07.2015
 *      Author: itsowl
 */

#include <src/IbeoScalaDataListener.hpp>
#include <iostream>

IbeoScalaDataListener::IbeoScalaDataListener(std::string ip, std::string scalaScope) : scalaIp(ip),
	msgIbeoScala(new rst::claas::IbeoScala_1403_Scan_2208),
	msgIbeoScalaObjectData_2271(new rst::claas::IbeoScala_1403_ObjectData_2271),
	msgIbeoScalaDeviceStatus(new rst::claas::IbeoScala_1403_DeviceStatus){

	rsb::Factory& factory = rsb::getFactory();
	informer = factory.createInformer< rst::claas::IbeoScala_1403_Scan_2208 > (scalaScope);
	informer_ObjectData_2271 = factory.createInformer<rst::claas::IbeoScala_1403_ObjectData_2271> (scalaScope);
	informer_DeviceStatus = factory.createInformer<rst::claas::IbeoScala_1403_DeviceStatus> (scalaScope);;
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

void IbeoScalaDataListener::onData(const ibeosdk::DeviceStatus* const deviceStatus){
	msgIbeoScalaDeviceStatus->set_uniqueidentifier(scalaIp);
	msgIbeoScalaDeviceStatus->set_serial_year(deviceStatus->getSerialNumber().getYear());
	msgIbeoScalaDeviceStatus->set_serial_month(deviceStatus->getSerialNumber().getMonth());
	msgIbeoScalaDeviceStatus->set_serial_cnt1(deviceStatus->getSerialNumber().getCnt1());
	msgIbeoScalaDeviceStatus->set_serial_cnt0(deviceStatus->getSerialNumber().getCnt0());
	msgIbeoScalaDeviceStatus->set_serial_null(deviceStatus->getSerialNumber().getNull());
	msgIbeoScalaDeviceStatus->set_fpgaversion(deviceStatus->getFpgaVersion().getVersion());
	msgIbeoScalaDeviceStatus->set_dspversion(deviceStatus->getDspVersion().getVersion());
	msgIbeoScalaDeviceStatus->set_hostversion(deviceStatus->getHostVersion().getVersion());
	msgIbeoScalaDeviceStatus->set_fpgamodusregister(deviceStatus->getFpgaModusRegister());
	msgIbeoScalaDeviceStatus->set_sensortemperature(deviceStatus->getSensorTemperature());
	msgIbeoScalaDeviceStatus->set_frequency(deviceStatus->getFrequency());
	msgIbeoScalaDeviceStatus->set_apdtablevoltage(deviceStatus->getApdTableVoltage());
	msgIbeoScalaDeviceStatus->set_noisemeasurementthreshold(deviceStatus->getNoiseMeasurementThreshold());
	msgIbeoScalaDeviceStatus->set_referencenoise(deviceStatus->getReferenceNoise());
	msgIbeoScalaDeviceStatus->set_actualnoise(deviceStatus->getActualNoise(0));

	informer_DeviceStatus->publish(msgIbeoScalaDeviceStatus);
}

void IbeoScalaDataListener::onData(const ibeosdk::ObjectListScala2271* const objs) {
	msgIbeoScalaObjectData_2271->set_starttimestamp(objs->getScanStartTimestamp());
	msgIbeoScalaObjectData_2271->set_scannumber(objs->getScanNumber());
	msgIbeoScalaObjectData_2271->set_numberobjects(objs->getNumberOfObjects());

	//std::cout << "Number of Objets:  " << objs->getNumberOfObjects() << "\n";

	for (unsigned int i = 0; i < msgIbeoScalaObjectData_2271->numberobjects(); ++i){
		msgIbeoScalaObjectData_2271->add_object();
		msgIbeoScalaObjectData_2271->mutable_object(i)->set_objectid(objs->getObjects()[i].getObjectId());
		msgIbeoScalaObjectData_2271->mutable_object(i)->set_attributeflags(objs->getObjects()[i].getAttributeFlags());
		msgIbeoScalaObjectData_2271->mutable_object(i)->set_interfaceflags(objs->getObjects()[i].getInterfaceFlags());

		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->set_reltimemeasure(objs->getObjects()[i].getUnfilteredObjectAttributes().getRelativeTimeOfMeasure());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_point_positionclosest()->set_x(objs->getObjects()[i].getUnfilteredObjectAttributes().getPositionClosestObjectPoint().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_point_positionclosest()->set_y(objs->getObjects()[i].getUnfilteredObjectAttributes().getPositionClosestObjectPoint().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_point_objectboxsize()->set_x(objs->getObjects()[i].getUnfilteredObjectAttributes().getObjectBoxSize().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_point_objectboxsize()->set_y(objs->getObjects()[i].getUnfilteredObjectAttributes().getObjectBoxSize().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_point_objectboxsizesigma()->set_x(objs->getObjects()[i].getUnfilteredObjectAttributes().getObjectBoxSizeSigma().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_point_objectboxsizesigma()->set_y(objs->getObjects()[i].getUnfilteredObjectAttributes().getObjectBoxSizeSigma().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->set_objectboxorientation(objs->getObjects()[i].getUnfilteredObjectAttributes().getObjectBoxOrientation());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->set_objectboxorientationsigma(objs->getObjects()[i].getUnfilteredObjectAttributes().getObjectBoxOrientationSigma());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_trackingcoordinate()->set_x(objs->getObjects()[i].getUnfilteredObjectAttributes().getReferencePointCoord().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_trackingcoordinate()->set_y(objs->getObjects()[i].getUnfilteredObjectAttributes().getReferencePointCoord().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_trackingcoordinatesigma()->set_x(objs->getObjects()[i].getUnfilteredObjectAttributes().getReferencePointCoordSigma().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_trackingcoordinatesigma()->set_y(objs->getObjects()[i].getUnfilteredObjectAttributes().getReferencePointCoordSigma().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->set_numbercontourpoints(objs->getObjects()[i].getUnfilteredObjectAttributes().getPossibleNbOfContourPoints());

		for (unsigned j = 0 ; j < msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->numbercontourpoints(); ++j){
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->add_contourpoints();
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_contourpoints(j)->set_point_x(objs->getObjects()[i].getUnfilteredObjectAttributes().getContourPoints()[j].getX());
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_contourpoints(j)->set_point_y(objs->getObjects()[i].getUnfilteredObjectAttributes().getContourPoints()[j].getX());
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_contourpoints(j)->set_point_x_sigma(objs->getObjects()[i].getUnfilteredObjectAttributes().getContourPoints()[j].getXSigma());
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_untackedproperties()->mutable_contourpoints(j)->set_point_y_sigma(objs->getObjects()[i].getUnfilteredObjectAttributes().getContourPoints()[j].getYSigma());
		}

		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_objectage(objs->getObjects()[i].getFilteredObjectAttributes().getObjectAge());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_hiddenstatusage(objs->getObjects()[i].getFilteredObjectAttributes().getHiddenStatusAge());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_dynamicflags(objs->getObjects()[i].getFilteredObjectAttributes().getDynamicFlag());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_reltimemeasure(objs->getObjects()[i].getFilteredObjectAttributes().getRelativeTimeOfMeasure());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_positionclosest()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getPositionClosestObjectPoint().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_positionclosest()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getPositionClosestObjectPoint().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_relvelocity()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getRelativeVelocity().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_relvelocity()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getRelativeVelocity().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_relvelocitysigma()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getRelativeVelocitySigma().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_relvelocitysigma()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getRelativeVelocitySigma().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_objectclaas(objs->getObjects()[i].getFilteredObjectAttributes().getClassification());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_classificationage(objs->getObjects()[i].getFilteredObjectAttributes().getClassificationAge());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_objectboxsize()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getObjectBoxSize().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_objectboxsize()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getObjectBoxSize().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_objectboxsizesigma()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getObjectBoxSizeSigma().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_objectboxsizesigma()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getObjectBoxSizeSigma().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_trackpointlocation((rst::claas::TrackingPointLocation)objs->getObjects()[i].getFilteredObjectAttributes().getReferencePointLocation());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_objectboxorientation(objs->getObjects()[i].getFilteredObjectAttributes().getObjectBoxOrientation());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_objectboxorientationsigma(objs->getObjects()[i].getFilteredObjectAttributes().getObjectBoxOrientationSigma());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_trackcoordinate()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getReferencePointCoord().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_trackcoordinate()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getReferencePointCoord().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_trackcoordinatesigma()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getReferencePointCoordSigma().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_trackcoordinatesigma()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getReferencePointCoordSigma().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_velocity()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getAbsoluteVelocity().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_velocity()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getAbsoluteVelocity().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_velocitysigma()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getAbsoluteVelocitySigma().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_velocitysigma()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getAbsoluteVelocitySigma().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_acceleration()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getAcceleration().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_acceleration()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getAcceleration().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_accelerationsigma()->set_x(objs->getObjects()[i].getFilteredObjectAttributes().getAccelerationSigma().getX());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_point_accelerationsigma()->set_y(objs->getObjects()[i].getFilteredObjectAttributes().getAccelerationSigma().getY());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_yawrate(objs->getObjects()[i].getFilteredObjectAttributes().getYawRate());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_yawratesigma(objs->getObjects()[i].getFilteredObjectAttributes().getYawRateSigma());
		msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->set_numbercontourpoints(objs->getObjects()[i].getFilteredObjectAttributes().getPossibleNbOfContourPoints());

		for (unsigned k = 0 ; k < msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->numbercontourpoints(); ++k){
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->add_contourpoints();
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_contourpoints(k)->set_point_x(objs->getObjects()[i].getFilteredObjectAttributes().getContourPoints()[k].getX());
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_contourpoints(k)->set_point_y(objs->getObjects()[i].getFilteredObjectAttributes().getContourPoints()[k].getX());
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_contourpoints(k)->set_point_x_sigma(objs->getObjects()[i].getFilteredObjectAttributes().getContourPoints()[k].getXSigma());
			msgIbeoScalaObjectData_2271->mutable_object(i)->mutable_trackedproperties()->mutable_contourpoints(k)->set_point_y_sigma(objs->getObjects()[i].getFilteredObjectAttributes().getContourPoints()[k].getYSigma());
		}

	}
	//std::cout <<"Objects in Proto:  " << msgIbeoScalaObjectData_2271->mutable_object()->size() << "\n";

	informer_ObjectData_2271->publish(msgIbeoScalaObjectData_2271);
	msgIbeoScalaObjectData_2271->Clear();
}
