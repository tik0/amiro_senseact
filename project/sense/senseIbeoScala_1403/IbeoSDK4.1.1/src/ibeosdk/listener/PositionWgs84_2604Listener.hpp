//======================================================================
/*! \file PositionWgs84_2604Listener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Jan 24, 2014
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_POSITIONWGS84_2604LISTENER_HPP_SEEN
#define IBEOSDK_POSITIONWGS84_2604LISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/PositionWgs84_2604.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

//======================================================================
/*!\brief Abstract base class for all DataListener listen on PositionWgs84_2604.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<PositionWgs84_2604> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_PositionWGS84 as
	 *        associated DataType.
	 * \return Always ibeo#DataType_PositionWGS84.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return PositionWgs84_2604::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new PositionWgs84_2604 DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type PositionWgs84_2604
	 * has been received.
	 *
	 * \param[in] wgs84  Pointer to the PositionWgs84_2604 that has
	 *                   been received.
	 *///-------------------------------------
	virtual void onData(const PositionWgs84_2604* const wgs84) = 0;
}; // PositionWgs84Listener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_POSITIONWGS84_2604LISTENER_HPP_SEEN

//======================================================================
