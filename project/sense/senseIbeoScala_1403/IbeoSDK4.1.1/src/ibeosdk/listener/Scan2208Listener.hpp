//======================================================================
/*! \file Scan2208Listener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Oct 04, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_SCAN2208LISTENER_HPP_SEEN
#define IBEOSDK_SCAN2208LISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/Scan2208.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on ScanLUX.
 * \author  Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Oct 04, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<Scan2208> : public DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_Scan2208 as
	 *        associated DataType.
	 * \return Always ibeo#DataType_Scan2208.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return Scan2208::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new Scan2208 DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type Scan2208
	 * has been received.
	 *
	 * \param[in] scan  Pointer to the scan that has
	 *                  been received.
	 *///-------------------------------------
	virtual void onData(const Scan2208* const scan) = 0;
}; // Scan2208Listener

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_SCAN2208LISTENER_HPP_SEEN

//======================================================================

