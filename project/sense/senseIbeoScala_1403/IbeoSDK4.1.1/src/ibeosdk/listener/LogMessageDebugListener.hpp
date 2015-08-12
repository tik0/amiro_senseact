//======================================================================
/*! \file LogMessageDebugListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_LOGMESSAGEDEBUGLISTENER_HPP_SEEN
#define IBEOSDK_LOGMESSAGEDEBUGLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/LogMessageDebug.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on LogMessageDebug.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<LogMessageDebug> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_LogDebug as
	 *        associated DataType.
	 * \return Always ibeo#DataType_LogDebug.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return LogMessageDebug::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new LogMessageDebug DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type LogMessageDebug
	 * has been received.
	 *
	 * \param[in] logMsg  Pointer to the scan that has
	 *                    been received.
	 *///-------------------------------------
	virtual void onData(const LogMessageDebug* const logMsg) = 0;
}; // LogMessageDebugListener

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_LOGMESSAGEERRORLISTENER_HPP_SEEN

//======================================================================