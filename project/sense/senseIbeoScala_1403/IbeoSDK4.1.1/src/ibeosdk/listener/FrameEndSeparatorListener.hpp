//======================================================================
/*! \file FrameEndSeparatorListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Nov 26, 2013
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_FRAMEENDSEPARATORLISTENER_HPP_SEEN
#define IBEOSDK_FRAMEENDSEPARATORLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/FrameEndSeparator.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

//======================================================================
/*!\brief Abstract base class for all DataListener listen on FrameEndSeparator.
 * \author Jan Christian Dittmer
 * \version 0.1
 * \date Nov 4, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<FrameEndSeparator> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_FrameEndSeparator as
	 *        associated DataType.
	 * \return Always ibeo#DataType_FrameEndSeparator.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return FrameEndSeparator::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new FrameEndSeparator DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type FrameEndSeparator
	 * has been received.
	 *
	 * \param[in] fes  Pointer to the FrameEndSeparator that has
	 *                 been received.
	 *///-------------------------------------
	virtual void onData(const FrameEndSeparator* const fes) = 0;
}; // FrameEndSeparatorListener

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_FRAMEENDSEPARATORLISTENER_HPP_SEEN

//======================================================================
