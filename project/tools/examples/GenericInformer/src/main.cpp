#include "Watchdog.pb.h"
#include "lase_2000D_226.pb.h"
#include "GenericInformer.hpp"



int main(int argc, char* argv[])
{
	GenericInformer<rst::watchdog::Watchdog> genericinformer("/scope");

	genericinformer.getMsgPointer()->set_spread(1);
	genericinformer.getMsgPointer()->set_rsb_ws_bridge_claas(1);
	genericinformer.getMsgPointer()->set_rsb_loggercpp_0_11(1);
	genericinformer.getMsgPointer()->set_bag_record(1);

	genericinformer.publishMsg();

	GenericInformer<rst::claas::LASE_2000D_226> genericInformerLase("/LASE");
	genericInformerLase.getMsgPointer()->set_scannumber(1);
	genericInformerLase.getMsgPointer()->set_timestamp(2);
	genericInformerLase.getMsgPointer()->set_systemtemperature(3);
	genericInformerLase.getMsgPointer()->add_distance(4);
	genericInformerLase.getMsgPointer()->add_pulsewidth(5);
	genericInformerLase.getMsgPointer()->set_uniqueidentifier("Temp");

	genericInformerLase.publishMsg();

}
