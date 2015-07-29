//======================================================================
/*! \file DeviceStatusListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Nov 4, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_DEVICESTATUSLISTENER_HPP_SEEN
#define IBEOSDK_DEVICESTATUSLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/DeviceStatus.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

//======================================================================
/*!\brief Abstract base class for all DataListener listen on DeviceStatus.
 * \author Jan Christian Dittmer
 * \version 0.1
 * \date Nov 4, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<DeviceStatus> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_DeviceStatus as
	 *        associated DataType.
	 * \return Always ibeo#DataType_DeviceStatus.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return DeviceStatus::getDataBlockId(); }

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
	virtual void onData(const DeviceStatus* const devStat) = 0;
}; // DeviceStatusListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_DEVICESTATUSLISTENER_HPP_SEEN

//======================================================================
