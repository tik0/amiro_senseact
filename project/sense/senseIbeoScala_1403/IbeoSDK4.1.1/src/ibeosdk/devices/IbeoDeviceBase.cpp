//======================================================================
/*! \file IbeoDeviceBase.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 7, 2015
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/devices/IbeoDeviceBase.hpp>

#include <ibeosdk/listener/DataStreamer.hpp>
#include <ibeosdk/listener/DataListener.hpp>
#include <ibeosdk/LogFileManager.hpp>

#include <ibeosdk/Log.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

IbeoDeviceBase::IbeoDeviceBase()
  : m_listeners(),
    m_threadState(TS_NotRunning),
    m_thread(NULL),
    m_logFileManager(NULL)
{}

//======================================================================

IbeoDeviceBase::~IbeoDeviceBase()
{
	this->disconnect();
}

//======================================================================

void IbeoDeviceBase::disconnect()
{
	if (this->m_thread) {
		{
			Lock lock(m_mutex);
			this->m_threadState = TS_Stopping;
		}
		m_recvCondition.notify_all();
		this->unregisterAll();

		if (this->m_thread->joinable()) {
			this->m_thread->join();
			delete this->m_thread;
			this->m_thread = NULL;
		}
	}
	else
		this->unregisterAll();
}

//======================================================================

bool IbeoDeviceBase::isConnected()
{
	return (this->m_threadState == TS_Running);
}

//======================================================================

void IbeoDeviceBase::setLogFileManager(LogFileManager* const logFileManager)
{
	this->m_logFileManager = logFileManager;
}

//======================================================================

void IbeoDeviceBase::unregisterAll()
{
	this->m_listeners.clear();
	this->m_streamers.clear();
}

//======================================================================

void IbeoDeviceBase::notifyStreamers(const IbeoDataHeader& dh, const char* const bodyBuf)
{
	if (bodyBuf == NULL)
		return;

	if (this->m_threadState != TS_Running)
		return;

	if (this->m_logFileManager) {
		this->m_logFileManager->checkSplitRequired(dh.getDataType());
	}

	if (this->m_threadState != TS_Running)
		return;

	std::list<DataStreamer*>::iterator streamerIter = m_streamers.begin();
	for (; streamerIter != m_streamers.end(); ++streamerIter) {
		(*streamerIter)->onData(dh, bodyBuf);
		if (this->m_threadState != TS_Running)
			return;
	}
}

//======================================================================

void IbeoDeviceBase::notifyListeners(const DataBlock* const data)
{
	if (data == NULL)
		return;

	if (this->m_threadState != TS_Running)
		return;

	if (this->m_logFileManager) {
		this->m_logFileManager->checkSplitRequired(data->getDataType());
	}

	ListenerListMap::iterator list = m_listeners.find(data->getDataType());
	if (list != m_listeners.end()) {
		ListenerList& listeners = list->second;
		ListenerList::iterator listenerIter = listeners.begin();
		for (; listenerIter != listeners.end(); ++listenerIter) {
			data->callListener(**listenerIter);
			if (this->m_threadState != TS_Running)
				return;
		} // for listenerIter
	} // if list exists
}

//======================================================================

void IbeoDeviceBase::registerListener(DataListenerBase* const listener)
{
	m_listeners[listener->getAssociatedDataType()].push_back(listener);
}

//======================================================================

statuscodes::Codes IbeoDeviceBase::unregisterListener(DataListenerBase* const listener)
{
	ListenerListMap::iterator llmIter = m_listeners.find(listener->getAssociatedDataType());
	if (llmIter == m_listeners.end())
		return statuscodes::ListenerNotRegistered;

	ListenerList& list = llmIter->second;

	ListenerList::iterator listenerIter = std::find(list.begin(), list.end(), listener);
	if (listenerIter == list.end())
		return statuscodes::ListenerNotRegistered;

	list.erase(listenerIter);
	return statuscodes::EverythingOk;
}

//======================================================================

void IbeoDeviceBase::registerStreamer(DataStreamer* const streamer)
{
	this->m_streamers.push_back(streamer);
}

//======================================================================

statuscodes::Codes IbeoDeviceBase::unregisterStreamer(DataStreamer* const streamer)
{
	std::list<DataStreamer*>::iterator steamerIter = std::find(m_streamers.begin(), m_streamers.end(), streamer);
	if (steamerIter == m_streamers.end())
		return statuscodes::StreamerNotRegistered;

	m_streamers.erase(steamerIter);
	return statuscodes::EverythingOk;
}

//======================================================================

const DataBlock* IbeoDeviceBase::deserialize(std::istream& is,
                                                  DataBlock& db,
                                                  const IbeoDataHeader& dh)
{
	if (db.getDataType() != dh.getDataType()) {
		logError << std::setfill('0')
			<< "Tried to deserialize a "
			<< "0x" << std::hex << std::setw(4) << dh.getDataType()
			<< " DataBlock into a "
			<< "0x" << std::hex << std::setw(4) << db.getDataType()
			<< " object." << std::dec << std::endl;
			return NULL;
	}

	db.setDataHeader(dh);
	if (db.deserialize(is, dh)) {
		logDebug2 << "Sucessfully deserializated data type " << toHex(uint16_t(dh.getDataType()))
				 << std::endl;
		return &db;
	}
	logError << "Failed to deserialization data type "
			<< toHex(uint16_t(dh.getDataType()))
			<< "  size: " << dh.getMessageSize() << " bytes"
			<< std::endl;
	return NULL;
}

//======================================================================

std::ostream& operator<<(std::ostream& os, const statuscodes::Codes ec)
{
	switch (ec) {
	case statuscodes::EverythingOk:             os << "EC: [Everything Ok]"; break;
	case statuscodes::NotConnected:             os << "EC: [Not Connected]"; break;
	case statuscodes::MismatchingCommandAndReply:    os << "EC: Provide the wrong reply container for command."; break;
	case statuscodes::FailedToPrepareSendingCommand: os << "EC: [Failed To Prepare Sending Command]"; break;
	case statuscodes::SendingCommandFailed:     os << "EC: [Sending Command Failed]"; break;
	case statuscodes::ReplyMismatch:            os << "EC: [Received the reply for another (unexpected) command]"; break;
	case statuscodes::TimeOut:                  os << "EC: [TimeOut]"; break;
	case statuscodes::TimeOutCriticalSection:   os << "EC: [TimeOut Critical Section]"; break;
	case statuscodes::ReceiveCommandErrorReply: os << "EC: [Received Command Error Reply]"; break;
	case statuscodes::StreamerNotRegistered:    os << "EC: [Streamer not registered]"; break;
	case statuscodes::ListenerNotRegistered:    os << "EC: [Listener not registered]"; break;
	default:
		os << "EC: [<<Missing text for ec " << int(ec) << ">>]"; break;
	}
	return os;
}

//======================================================================

}// namespace ibeosdk

//======================================================================
