//======================================================================
/*! \file ObjectListEcuEtDynListener.hpp
 *
 * \copydoc Copyright
 * \author Kai-Uwe von Deylen (kd)
 * \date Mar 14, 2014
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_OBJECTLISTECUETDYNLISTENER_HPP_SEEN
#define IBEOSDK_OBJECTLISTECUETDYNLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/ObjectListEcuEtDyn.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on ObjectListEcuEtDyn.
 * \author Kai-Uwe von Deylen (kd)
 * \version 0.1
 * \date Mar 14, 2014
 *///-------------------------------------------------------------------
template<>
class DataListener<ObjectListEcuEtDyn> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_EcuObjectListETDyn as
	 *        associated DataType.
	 * \return Always ibeo#DataType_EcuObjectListETDyn.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return ObjectListEcuEtDyn::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new ObjectListEcuEtDyn DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type ObjectListEcuEtDyn
	 * has been received.
	 *
	 * \param[in] objectList  Pointer to the object list that has
	 *                        been received.
	 *///-------------------------------------
	virtual void onData(const ObjectListEcuEtDyn* const objectList) = 0;
}; // ObjectListEcuEtDynListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_OBJECTLISTECUETDYNLISTENER_HPP_SEEN

//======================================================================

