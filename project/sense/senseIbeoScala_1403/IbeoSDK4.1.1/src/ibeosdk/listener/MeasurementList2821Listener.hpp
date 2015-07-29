//======================================================================
/*! \file MeasurementList2821Listener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Aug 11, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_MEASUREMENTLIST2821LISTENER_HPP_SEEN
#define IBEOSDK_MEASUREMENTLIST2821LISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/MeasurementList2821.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on MeasurementList2821.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Aug 11, 2014
 *///-------------------------------------------------------------------
template<>
class DataListener<MeasurementList2821> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_MeasurementList2821 as
	 *        associated DataType.
	 * \return Always ibeo#DataType_MeasurementList2821.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return MeasurementList2821::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new MeasurementList2821 DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type MeasurementList2821
	 * has been received.
	 *
	 * \param[in] measurementList  Pointer to the MeasurementList2821 that has
	 *                             been received.
	 *///-------------------------------------
	virtual void onData(const MeasurementList2821* const measurementList) = 0;
}; // ObjectListEcuEtDynListener

//======================================================================

}// namespace ibeosdk


//======================================================================

#endif // IBEOSDK_MEASUREMENTLIST2821LISTENER_HPP_SEEN

//======================================================================
