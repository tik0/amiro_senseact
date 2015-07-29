//======================================================================
/*! \file ObjectListLuxListener.hpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date Apr 30, 2012
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_OBJECTLISTLUXLISTENER_HPP_SEEN
#define IBEOSDK_OBJECTLISTLUXLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/ObjectListLux.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on ObjectListLux.
 * \author Mario Brumm (mb)
 * \version 0.1
 * \date Apr 30, 2012
 *///-------------------------------------------------------------------
template<>
class DataListener<ObjectListLux> : public DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_LuxObjectList as
	 *        associated DataType.
	 * \return Always ibeo#DataType_LuxObjectList.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return ObjectListLux::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new ObjectListLux DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type ObjectListLux
	 * has been received.
	 *
	 * \param[in] objectList  Pointer to the object list that has
	 *                        been received.
	 *///-------------------------------------
	virtual void onData(const ObjectListLux* const objectList) = 0;
}; // ObjectListLuxListener

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_OBJECTLISTLUXLISTENER_HPP_SEEN

//======================================================================

