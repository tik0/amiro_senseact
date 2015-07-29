//======================================================================
/*! \file ScanEcuListener.hpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date May 03, 2012
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_SCANECULISTENER_HPP_SEEN
#define IBEOSDK_SCANECULISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/ScanEcu.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on ScanEcu.
 * \author Mario Brumm (mb)
 * \version 0.1
 * \date May 3, 2012
 *///-------------------------------------------------------------------
template<>
class DataListener<ScanEcu>: public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_EcuScan as
	 *        associated DataType.
	 * \return Always ibeo#DataType_EcuScan.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return ScanEcu::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new ScanEcu DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type ScanEcu
	 * has been received.
	 *
	 * \param[in] scan  Pointer to the scan that has
	 *                  been received.
	 *///-------------------------------------
	virtual void onData(const ScanEcu* const scan) = 0;
}; // ScanEcuListener

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_SCANECULISTENER_HPP_SEEN

//======================================================================

