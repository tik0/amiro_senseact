//======================================================================
/*! \file LogMessageNoteListener.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_LOGMESSAGENOTELISTENER_HPP_SEEN
#define IBEOSDK_LOGMESSAGENOTELISTENER_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/listener/DataListener.hpp>

#include <ibeosdk/datablocks/LogMessageNote.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================
/*!\brief Abstract base class for all DataListener listen on LogMessageNote.
 * \author Jan Christian Dittmer (jcd)
 * \version 0.1
 * \date Sep 30, 2013
 *///-------------------------------------------------------------------
template<>
class DataListener<LogMessageNote> : public ibeosdk::DataListenerBase {
public:
	//========================================
	/*!\brief Get ibeo#DataType_LogNote as
	 *        associated DataType.
	 * \return Always ibeo#DataType_LogNote.
	 *///-------------------------------------
	virtual DataTypeId getAssociatedDataType() const { return LogMessageNote::getDataBlockId(); }

	//========================================
	/*!\brief Called on receiving a new LogMessageNote DataBlock.
	 *
	 * Method to be called by the IbeoDevice where this listener
	 * is registered when a new DataBlock of type LogMessageNote
	 * has been received.
	 *
	 * \param[in] logMsg  Pointer to the scan that has
	 *                    been received.
	 *///-------------------------------------
	virtual void onData(const LogMessageNote* const logMsg) = 0;
}; // LogMessageNoteListener

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_LOGMESSAGENOTELISTENER_HPP_SEEN

//======================================================================
