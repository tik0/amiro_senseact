
#define INFO_MSG_
#define DEBUG_MSG_
// #define SUCCESS_MSG_
// #define WARNING_MSG_
#define ERROR_MSG_
#include <MSG.h>

#include <iostream>
#include <string>
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
#include <CanMessage.pb.h>

#include <boost/program_options.hpp>
#include <boost/make_shared.hpp>

//#include <Converter_ClaasCan_CombineHarvester.hpp>
#include <rsb/MetaData.h>

#include "Odemetrie.h" // Model's header file
#include <ClaasCan_CombineHarvester_mac.h>

std::string rsbOutScope = "/plan/Fahrzeugdaten";
rsb::Informer<rst::claas::MachineModel_Odemetrie>::DataPtr msgOdemetrie;
rsb::Informer<rst::claas::MachineModel_Odemetrie >::Ptr informerMachineModel;

static OdemetrieModelClass Odemetrie_Obj;// Instance of model class

void machineStep(){
	static boolean_T OverrunFlag = 0;

	// Disable interrupts here

	// Check for overrun
	if (OverrunFlag) {
	rtmSetErrorStatus(Odemetrie_Obj.getRTM(), "Overrun");
	return;
	}

	OverrunFlag = true;
	// Save FPU context here (if necessary)
	// Re-enable timer or interrupt here
	// Set model inputs here

	// Step the model
	Odemetrie_Obj.step();
	/* Get model outputs here */
	msgOdemetrie->set_phi_kon(Odemetrie_Obj.Odemetrie_Y.Phi_kon); // '<Root>/Phi_kon'
	msgOdemetrie->set_y_dis_roi(Odemetrie_Obj.Odemetrie_Y.Y_dis_roi);      // '<Root>/Y_dis_roi'
	msgOdemetrie->set_x_dis_roi(Odemetrie_Obj.Odemetrie_Y.X_dis_roi);      // '<Root>/X_dis_roi'
	msgOdemetrie->set_center_dresch(Odemetrie_Obj.Odemetrie_Y.center_Dresch);  // '<Root>/center_Dresch'
	msgOdemetrie->set_x_dis_glo(Odemetrie_Obj.Odemetrie_Y.X_dis_glo);      // '<Root>/X_dis_glo'
	msgOdemetrie->set_y_dis_glo(Odemetrie_Obj.Odemetrie_Y.Y_dis_glo);      // '<Root>/Y_dis_glo'
	msgOdemetrie->set_x_kon_roi(Odemetrie_Obj.Odemetrie_Y.X_kon_roi);      // '<Root>/X_kon_roi'
	msgOdemetrie->set_x_kon_glo(Odemetrie_Obj.Odemetrie_Y.X_kon_glo);      // '<Root>/X_kon_glo'
	msgOdemetrie->set_y_kon_roi(Odemetrie_Obj.Odemetrie_Y.Y_kon_roi);      // '<Root>/Y_kon_roi'
	msgOdemetrie->set_y_kon_glo(Odemetrie_Obj.Odemetrie_Y.Y_kon_glo);	// '<Root>/Y_kon_glo'
	informerMachineModel->publish(msgOdemetrie);
	/* Indicate task complete */
	OverrunFlag = false;

	/* Disable interrupts here */
	/* Restore FPU context here (if necessary) */
	/* Enable interrupts here */
}

void getMachineModelData(rsb::EventPtr canEvent){

	boost::shared_ptr<rst::claas::CanMessage> sample_data =
			boost::static_pointer_cast<rst::claas::CanMessage>(canEvent->getData());

	unsigned char canData[8];
	for(unsigned int i = 0; i< sample_data->candlc(); ++i){
		canData[i] = sample_data->canmessage(i);
	}
	uint32_t canId = sample_data->canid();
	static short drivingDirChange = 1;
	if (canId ==  cVdtcShlcVehicleSpeedMta_ID){

		Odemetrie_Obj.Odemetrie_U.Machine_Speed = cVdtcShlcVehicleSpeedMta_VehicleSpeed_GETP(canData) * drivingDirChange;
		double sendTime = canEvent->getMetaData().getSendTime();
		static double lastSendTime = sendTime;

		if(rtmGetErrorStatus(Odemetrie_Obj.getRTM()) == NULL){
			Odemetrie_Obj.Odemetrie_U.tSample = (sendTime - lastSendTime) / 1000000;
			machineStep();
			lastSendTime = sendTime;
			//std::cout << "X_dis_roi: " << Odemetrie_Obj.Odemetrie_Y.X_dis_roi << "  Y_dis_roi: " << Odemetrie_Obj.Odemetrie_Y.Y_dis_roi << "  Phi_kon:  "<< Odemetrie_Obj.Odemetrie_Y.Phi_kon << " Speed: " << Odemetrie_Obj.Odemetrie_U.Machine_Speed << " EstimatedCurvature: " << Odemetrie_Obj.Odemetrie_U.Machine_EstimatedCurvature
			//<< "   SendTime: " <<canEvent->getMetaData().getSendTime() << "  tSample: " << Odemetrie_Obj.Odemetrie_U.tSample <<std::endl;
		}
	}
	else if (canId == cAtpcBrc_AtpStatus_ID) {

		Odemetrie_Obj.Odemetrie_U.Machine_EstimatedCurvature = cAtpcBrc_AtpStatus_EstimatedCurvature_GETP(canData);
	}
	else if (canId == cVdtcBrc_DrivingDirStatus_ID){
		short drivingDir = cVdtcBrc_DrivingDirStatus_DrivingDirStatusActVal_GETP(canData);
		//std::cout << "DrivingStatus: " << drivingDir << "\n";
		switch(drivingDir){
			case 0:
				drivingDirChange = 1;
				break;
			case 2:
				drivingDirChange = -1;
				break;
		}
	}
}

int main(int argc, char **argv) {

	std::string rsbInScope = "/sense/ClaasCan/rx";

	// Initialize model
	Odemetrie_Obj.initialize();
	Odemetrie_Obj.Odemetrie_U.Offsetcurvature = 0.0226;
	Odemetrie_Obj.Odemetrie_U.Diskrete_Auf = 0.05;
	Odemetrie_Obj.Odemetrie_U.tSample = 0.05;

	//Odemetrie_U.Offsetcurvature = 0.0226;
	//Odemetrie_U.Diskrete_Auf = 0.05;
	//Odemetrie_U.tSample = 0.05;

	INFO_MSG("")
	// Handle program options
	namespace po = boost::program_options;

	po::options_description options("Allowed options");
	options.add_options()("help,h", "Display a help message.")
	("outscope,o", po::value < std::string > (&rsbOutScope), "Scope for sending odometry values, dafault = \"/plan/Fahrzeugdaten\"")
	("offset,c", po::value < double > (&Odemetrie_Obj.Odemetrie_U.Offsetcurvature), "Offset curvature for machine, dafault = 0.0226")
	("diskretAuf,d", po::value < double > (&Odemetrie_Obj.Odemetrie_U.Diskrete_Auf), "Discrete resolution for roi = 0.05");
	// allow to give the value as a positional argument
	po::positional_options_description p;
	p.add("value", 1);

	po::variables_map vm;
	po::store( po::command_line_parser(argc, argv).options(options).positional(p).run(), vm);

	// first, process the help option
	if (vm.count("help")) {
	  std::cout << options << "\n";
	  exit(1);
	}

	// afterwards, let program options handle argument errors
	po::notify(vm);
	rsc::misc::initSignalWaiter();

	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::MachineModel_Odemetrie> >
	  converter(new rsb::converter::ProtocolBufferConverter<rst::claas::MachineModel_Odemetrie>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter);

	rsb::Factory& factory = rsb::getFactory();
	informerMachineModel = factory.createInformer< rst::claas::MachineModel_Odemetrie > (rsbOutScope);
	msgOdemetrie =  boost::make_shared<rst::claas::MachineModel_Odemetrie>();

	boost::shared_ptr< rsb::converter::ProtocolBufferConverter<rst::claas::CanMessage> >
	  converter_CanMsg(new rsb::converter::ProtocolBufferConverter<rst::claas::CanMessage>());
	rsb::converter::converterRepository<std::string>()->registerConverter(converter_CanMsg);

	rsb::ListenerPtr listener = factory.createListener(rsbInScope);
	listener->addHandler(rsb::HandlerPtr(new rsb::EventFunctionHandler(&getMachineModelData)));

	return rsc::misc::suggestedExitCode(rsc::misc::waitForSignal());

}

