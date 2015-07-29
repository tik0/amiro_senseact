//======================================================================
/*! \file ObjectListScala2271Listener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 24, 2014
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_OBJECTLISTSCALA2271LISTENER_HPP_SEEN
#define IBEOSDK_OBJECTLISTSCALA2271LISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/ObjectListScala2271.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
//======================================================================
/*!\brief Abstract base class for all DataListener listen on ObjectListScala2271.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<ObjectListScala2271> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_Image as
	 *        associated DataType.
	 * \return Always ibeo#DataType_Image.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return ObjectListScala2271::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new Image DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type Image
	 * has been received.
	 *
	 * \param[in] image  Pointer to the image that has
	 *                   been received.
	 *///-------------------------------------
	virtual void onData(const ObjectListScala2271* const image) = 0;
}; // ObjectListScala2271Listener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_OBJECTLISTSCALA2271LISTENER_HPP_SEEN

//======================================================================
