//======================================================================
/*! \file VehicleStateBasicEcu2806Listener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Jan 29, 2014
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_VEHICLESTATEBASICECU2806LISTENER_HPP_SEEN
#define IBEOSDK_VEHICLESTATEBASICECU2806LISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/VehicleStateBasicEcu2806.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!
 * \brief Abstract base class for all DataListener listen on VehicleStateBasicEcu2806.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Jan 29, 2014
 *///-------------------------------------------------------------------
template<>
class DataListener<VehicleStateBasicEcu2806> : public ibeosdk::DataListenerBase {
public:
		//========================================
		/*!\brief Get ibeo#DataType_EcuVehicleStateBasic2806 as
		 *        associated DataType.
		 * \return Always ibeo#DataType_EcuVehicleStateBasic2806.
		 *///-------------------------------------
		virtual DataTypeId getAssociatedDataType() const { return VehicleStateBasicEcu2806::getDataBlockId(); }

		//========================================
	/*!\brief Called on receiving a new VehicleStateBasicEcu2806 DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type VehicleStateBasicEcu2806
	 * has been received.
	 *
	 * \param[in] vsb  Pointer to the VehicleStateBasicEcu2806 that has
	 *                 been received.
	 *///-------------------------------------
	virtual void onData(const VehicleStateBasicEcu2806* const vsb) = 0;
}; // VehicleStateBasicEcu2806Listener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_VEHICLESTATEBASICECU2806LISTENER_HPP_SEEN

//======================================================================
