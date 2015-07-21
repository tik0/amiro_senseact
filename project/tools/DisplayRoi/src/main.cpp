
//#include <opencv2/core/core.hpp>
//#include <opencv2/highgui/highgui.hpp>

#include "Auswertung_initialize.h"
#include "Auswertung_terminate.h"
#include "Auswertung.h"
#include "Auswertung_data.h"

#include <thread>
#include <chrono>
#include <iostream>

//RSC
#include <rsc/misc/SignalWaiter.h>
// RSB
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rsb/Event.h>
#include <rsb/Handler.h>
// RST
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <MachineModel.pb.h>

double x_dis_r = 0;
double y_dis_r = 0;
double phi_kon = 0;

void getMachineModelData(rsb::EventPtr canEvent){

	boost::shared_ptr<rst::claas::MachineModel_Odemetrie> sample_data = boost::static_pointer_cast<rst::claas::MachineModel_Odemetrie>(canEvent->getData());
	x_dis_r = sample_data->x_dis_roi();
	y_dis_r = sample_data->y_dis_roi();
	phi_kon = sample_data->phi_kon();
	//std::cout<< "X_dis_roi: " <<sample_data->x_dis_roi() << "  Y_dis_roi: " << sample_data->y_dis_roi() << std::endl;
}

int main(int argc, char* argv[]) {

	std::string rsbInScope = "/plan/Fahrzeugdaten";
	Auswertung_initialize();

	rsc::misc::initSignalWaiter();

	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::MachineModel_Odemetrie> >
	  converter(new rsb::converter::ProtocolBufferConverter<rst::claas::MachineModel_Odemetrie>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	rsb::Factory& factory = rsb::getFactory();

	rsb::ListenerPtr listener = factory.createListener(rsbInScope);
	listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&getMachineModelData)));

	while(1){
		Auswertung(x_dis_r, y_dis_r, phi_kon, 0,0,0,10);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	Auswertung_terminate();

	return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());
}
