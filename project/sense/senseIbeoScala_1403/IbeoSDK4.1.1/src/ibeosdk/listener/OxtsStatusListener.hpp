//======================================================================
/*! \file OxtsStatusListener.hpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date Jun 30, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_OXTSSTATUSLISTENER_HPP_SEEN
#define IBEOSDK_OXTSSTATUSLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/OxtsStatus.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on OxtsStatus.
 * \author Kai-Uwe von Deylen (kd)
 * \version 0.1
 * \date Jun 30, 2014
 *///-------------------------------------------------------------------
template<>
class DataListener<OxtsStatus> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_OxtsStatus as
	 *        associated DataType.
	 * \return Always ibeo#DataType_OxtsStatus.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return OxtsStatus::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new OxtsStatus DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type OxtsStatus
	 * has been received.
	 *
	 * \param[in] oxtsStatus  Pointer to the OxtsStatus that has
	 *                        been received.
	 *///-------------------------------------
	virtual void onData(const OxtsStatus* const oxtsStatus) = 0;
}; // ObjectListEcuEtDynListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_OXTSSTATUSLISTENER_HPP_SEEN

//======================================================================

