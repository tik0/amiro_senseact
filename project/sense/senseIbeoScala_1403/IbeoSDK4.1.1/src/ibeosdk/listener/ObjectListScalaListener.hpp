//======================================================================
/*! \file ObjectListScalaListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Jan 17, 2014
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_OBJECTLISTSCALALISTENER_HPP_SEEN
#define IBEOSDK_OBJECTLISTSCALALISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/ObjectListScala.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on ObjectListScala.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Jan 17, 2014
 *///-------------------------------------------------------------------
template<>
class DataListener<ObjectListScala> : public DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_ScalaObjectList as
	 *        associated DataType.
	 * \return Always ibeo#DataType_ScalaObjectList.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return ObjectListScala::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new ObjectListScala DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type ObjectListScala
	 * has been received.
	 *
	 * \param[in] objectList  Pointer to the object list that has
	 *                        been received.
	 *///-------------------------------------
	virtual void onData(const ObjectListScala* const objectList) = 0;
}; // ObjectListScalaListener

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_OBJECTLISTSCALALISTENER_HPP_SEEN

//======================================================================

