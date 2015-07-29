//======================================================================
/*! \file CarriageWayList6970Listener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 29, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_CARRIAGEWAYLIST6970LISTENER_HPP_SEEN
#define IBEOSDK_CARRIAGEWAYLIST6970LISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/CarriageWayList6970.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

//======================================================================
/*!\brief Abstract base class for all DataListener listen on CarriageWayList6970.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<CarriageWayList6970> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get 0x6970 as
	 *        associated DataType.
	 * \return Always 0x6970.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return CarriageWayList6970::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new CarriageWayList6970 DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type CarriageWayList6970
	 * has been received.
	 *
	 * \param[in] cwl  Pointer to the CarriageWayList6970 that has
	 *                 been received.
	 *///-------------------------------------
	virtual void onData(const CarriageWayList6970* const cwl) = 0;
}; // CarriageWayList6970Listener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_CARRIAGEWAYLIST6970LISTENER_HPP_SEEN

//======================================================================
