//======================================================================
/*! \file ImageListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 29, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_IMAGELISTENER_HPP_SEEN
#define IBEOSDK_IMAGELISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/Image.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

//======================================================================
/*!\brief Abstract base class for all DataListener listen on Image.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<Image> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_Image as
	 *        associated DataType.
	 * \return Always ibeo#DataType_Image.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return Image::getDataBlockId(); }

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
	virtual void onData(const Image* const image) = 0;
}; // ImageListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_IMAGELISTENER_HPP_SEEN

//======================================================================
