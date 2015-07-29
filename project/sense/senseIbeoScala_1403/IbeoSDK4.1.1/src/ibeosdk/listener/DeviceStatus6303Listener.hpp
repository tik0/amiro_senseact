//======================================================================
/*! \file DeviceStatus6303Listener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Feb 20, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_DEVICESTATUS6303LISTENER_HPP_SEEN
#define IBEOSDK_DEVICESTATUS6303LISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/DeviceStatus6303.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

//======================================================================
/*!\brief Abstract base class for all DataListener listen on DeviceStatus6303.
 * \author Jan Christian Dittmer
 * \version 0.1
 * \date Feb 20, 2014
 *///-------------------------------------------------------------------
template<>
class DataListener<DeviceStatus6303> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_DeviceStatus as
	 *        associated DataType.
	 * \return Always ibeo#DataType_DeviceStatus.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return DeviceStatus6303::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new DeviceStatus DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type DeviceStatus
	 * has been received.
	 *
	 * \param[in] devStat  Pointer to the DeviceStatus that has
	 *                     been received.
	 *///-------------------------------------
	virtual void onData(const DeviceStatus6303* const devStat) = 0;
}; // DeviceStatus6303Listener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_DEVICESTATUS6303LISTENER_HPP_SEEN

//======================================================================
