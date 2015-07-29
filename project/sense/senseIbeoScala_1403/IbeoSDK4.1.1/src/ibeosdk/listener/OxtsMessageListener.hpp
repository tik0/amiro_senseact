//======================================================================
/*! \file OxtsMessageListener.hpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date Jun 30, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_OXTSMESSAGELISTENER_HPP_SEEN
#define IBEOSDK_OXTSMESSAGELISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/OxtsMessage.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on OxtsMessage.
 * \author Kai-Uwe von Deylen (kd)
 * \version 0.1
 * \date Jun 30, 2014
 *///-------------------------------------------------------------------
template<>
class DataListener<OxtsMessage> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_OxtsMessage as
	 *        associated DataType.
	 * \return Always ibeo#DataType_OxtsMessage.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return OxtsMessage::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new OxtsMessage DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type OxtsMessage
	 * has been received.
	 *
	 * \param[in] oxtsMessage  Pointer to the OxtsMessage that has
	 *                         been received.
	 *///-------------------------------------
	virtual void onData(const OxtsMessage* const oxtsMessage) = 0;
}; // ObjectListEcuEtDynListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_OXTSMESSAGELISTENER_HPP_SEEN

//======================================================================

