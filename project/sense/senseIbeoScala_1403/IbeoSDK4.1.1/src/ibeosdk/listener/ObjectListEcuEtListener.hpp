//======================================================================
/*! \file ObjectListEcuEtListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 05, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_OBJECTLISTECUETLISTENER_HPP_SEEN
#define IBEOSDK_OBJECTLISTECUETLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/ObjectListEcuEt.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on ObjectListEcuEt.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 5, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<ObjectListEcuEt> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_EcuObjectListET as
	 *        associated DataType.
	 * \return Always ibeo#DataType_EcuObjectListET.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return ObjectListEcuEt::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new ObjectListEcuEt DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type ObjectListEcuEt
	 * has been received.
	 *
	 * \param[in] objectList  Pointer to the object list that has
	 *                        been received.
	 *///-------------------------------------
	virtual void onData(const ObjectListEcuEt* const objectList) = 0;
}; // ObjectListEcuEtListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_OBJECTLISTECUETLISTENER_HPP_SEEN

//======================================================================

