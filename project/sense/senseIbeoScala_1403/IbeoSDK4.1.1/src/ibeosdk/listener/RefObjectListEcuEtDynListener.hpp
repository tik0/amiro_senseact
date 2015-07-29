//======================================================================
/*! \file RefObjectListEcuEtDynListener.hpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date Mar 14, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_REFOBJECTLISTECUETDYNLISTENER_HPP_SEEN
#define IBEOSDK_REFOBJECTLISTECUETDYNLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/RefObjectListEcuEtDyn.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on RefObjectListEcuEtDyn.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Mar 05, 2015
 *///-------------------------------------------------------------------
template<>
class DataListener<RefObjectListEcuEtDyn> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_ReferencesObjcListETDyn as
	 *        associated DataType.
	 * \return Always ibeo#DataType_ReferencesObjcListETDyn.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return RefObjectListEcuEtDyn::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new RefObjectListEcuEtDyn DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type RefObjectListEcuEtDyn
	 * has been received.
	 *
	 * \param[in] objectList  Pointer to the object list that has
	 *                        been received.
	 *///-------------------------------------
	virtual void onData(const RefObjectListEcuEtDyn* const objectList) = 0;
}; // RefObjectListEcuEtDynListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_REFOBJECTLISTECUETDYNLISTENER_HPP_SEEN

//======================================================================

