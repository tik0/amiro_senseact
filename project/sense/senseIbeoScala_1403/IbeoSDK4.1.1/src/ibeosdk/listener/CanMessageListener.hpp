//======================================================================
/*! \file CanMessageListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Jan 27, 2014
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_CANMESSAGELISTENER_HPP_SEEN
#define IBEOSDK_CANMESSAGELISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/CanMessage.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on CanMessage.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 2, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<CanMessage> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_CanMessage as
	 *        associated DataType.
	 * \return Always ibeo#DataType_CanMessage.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return CanMessage::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new CanMessage DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type CanMessage
	 * has been received.
	 *
	 * \param[in] canMsg  Pointer to the CanMessage that has
	 *                    been received.
	 *///-------------------------------------
	virtual void onData(const CanMessage* const canMsg) = 0;
}; // CanMessageListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_CANMESSAGELISTENER_HPP_SEEN

//======================================================================
