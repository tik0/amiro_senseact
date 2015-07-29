//======================================================================
/*! \file RefObjectListEcuEtListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 05, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_REFOBJECTLISTECUETLISTENER_HPP_SEEN
#define IBEOSDK_REFOBJECTLISTECUETLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/RefObjectListEcuEt.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on RefObjectListEcuEt.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 5, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<RefObjectListEcuEt> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_ReferencesObjcListET as
	 *        associated DataType.
	 * \return Always ibeo#DataType_ReferencesObjcListET.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return RefObjectListEcuEt::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new RefbjectListEcuEt DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type RefObjectListEcuEt
	 * has been received.
	 *
	 * \param[in] objectList  Pointer to the object list that has
	 *                        been received.
	 *///-------------------------------------
	virtual void onData(const RefObjectListEcuEt* const objectList) = 0;
}; // RefObjectListEcuEtListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_REFOBJECTLISTECUETLISTENER_HPP_SEEN

//======================================================================

