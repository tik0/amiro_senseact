//======================================================================
/*! \file VehicleStateBasicEcuListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 2, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_VEHICLESTATEBASICECULISTENER_HPP_SEEN
#define IBEOSDK_VEHICLESTATEBASICECULISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/VehicleStateBasicEcu.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!
 * \brief Abstract base class for all DataListener listen on VehicleStateBasicEcu.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 2, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<VehicleStateBasicEcu> : public DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_EcuVehicleStateBasic as
	 *        associated DataType.
	 * \return Always ibeo#DataType_EcuVehicleStateBasic.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return VehicleStateBasicEcu::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new VehicleStateBasicEcu DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type VehicleStateBasicEcu
	 * has been received.
	 *
	 * \param[in] vsb  Pointer to the vehicle state that has
	 *                 been received.
	 *///-------------------------------------
	virtual void onData(const VehicleStateBasicEcu* const vsb) = 0;
}; // VehicleStateBasicEcuListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_VEHICLESTATEBASICECULISTENER_HPP_SEEN

//======================================================================

