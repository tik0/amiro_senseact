//======================================================================
/*! \file LogMessageWarningListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_LOGMESSAGEWARNINGLISTENER_HPP_SEEN
#define IBEOSDK_LOGMESSAGEWARNINGLISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/LogMessageWarning.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on LogMessageWarning.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<LogMessageWarning> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_LogWarning as
	 *        associated DataType.
	 * \return Always ibeo#DataType_LogWarning.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return LogMessageWarning::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new LogMessageWarning DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type LogMessageWarning
	 * has been received.
	 *
	 * \param[in] logMsg  Pointer to the scan that has
	 *                    been received.
	 *///-------------------------------------
	virtual void onData(const LogMessageWarning* const logMsg) = 0;
}; // LogMessageWarningListener

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_LOGMESSAGEWARNINGLISTENER_HPP_SEEN

//======================================================================
