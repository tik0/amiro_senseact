//======================================================================
/*! \file LogMessageErrorListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_LOGMESSAGEERRORLISTENER_HPP_SEEN
#define IBEOSDK_LOGMESSAGEERRORLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/LogMessageError.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on LogMessageError.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<LogMessageError> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_LogError as
	 *        associated DataType.
	 * \return Always ibeo#DataType_LogError.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return LogMessageError::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new LogMessageError DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type LogMessageError
	 * has been received.
	 *
	 * \param[in] logMsg  Pointer to the scan that has
	 *                    been received.
	 *///-------------------------------------
	virtual void onData(const LogMessageError* const logMsg) = 0;
}; // LogMessageErrorListener

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_LOGMESSAGEERRORLISTENER_HPP_SEEN

//======================================================================
