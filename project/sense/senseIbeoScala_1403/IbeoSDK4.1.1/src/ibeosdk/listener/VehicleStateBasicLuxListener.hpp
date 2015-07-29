//======================================================================
/*! \file VehicleStateBasicLuxListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 2, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_VEHICLESTATEBASICLUXLISTENER_HPP_SEEN
#define IBEOSDK_VEHICLESTATEBASICLUXLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/VehicleStateBasicLux.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on VehicleStateBasicLux.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 2, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<VehicleStateBasicLux> : public DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_LuxVehicleStateBasic as
	 *        associated DataType.
	 * \return Always ibeo#DataType_LuxVehicleStateBasic.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return VehicleStateBasicLux::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new VehicleStateBasicLux DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type VehicleStateBasicLux
	 * has been received.
	 *
	 * \param[in] vsb  Pointer to the vehicle state that has
	 *                 been received.
	 *///-------------------------------------
	virtual void onData(const VehicleStateBasicLux* const vsb) = 0;
}; // VehicleStateBasicLuxListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_VEHICLESTATEBASICLUXLISTENER_HPP_SEEN

//======================================================================

