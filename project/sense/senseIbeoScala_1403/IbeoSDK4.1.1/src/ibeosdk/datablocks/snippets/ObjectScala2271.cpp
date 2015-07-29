//======================================================================
/*! \file ObjectScala2271.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 24, 2014
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/snippets/ObjectScala2271.hpp>
#include <ibeosdk/Log.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

ObjectScala2271::ObjectScala2271()
  : m_objectId(0),
    m_internal(0),
    m_interfaceFlags(0),
    m_attributeFlags(0),
    m_unfilteredAttrib(),
    m_filteredAttrib(),
    m_reserved(0)
{}

//======================================================================

ObjectScala2271::~ObjectScala2271() {}

//======================================================================

std::streamsize ObjectScala2271::getSerializedSize() const
{
	return std::streamsize(sizeof(uint32_t))
		+ 3* std::streamsize(sizeof(uint8_t))
		+ m_unfilteredAttrib.getSerializedSize()
		+ m_filteredAttrib.getSerializedSize()
		+ std::streamsize(sizeof(uint32_t));
}

//======================================================================

bool ObjectScala2271::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	ibeosdk::readBE(is, m_objectId);
	ibeosdk::readBE(is, m_internal);
	ibeosdk::readBE(is, m_interfaceFlags);
	ibeosdk::readBE(is, m_attributeFlags);

	m_unfilteredAttrib.setIsValid(this->hasUnfilteredAttributes());
	m_unfilteredAttrib.setHasContourPoints(this->hasUnfilteredContours());
	if (!m_unfilteredAttrib.deserialize(is))
		return false;

	m_filteredAttrib.setIsValid(this->hasFilteredAttributes());
	m_filteredAttrib.setHasContourPoints(this->hasFilteredContours());
	m_filteredAttrib.deserialize(is);

	ibeosdk::readBE(is, m_reserved);
	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//======================================================================

bool ObjectScala2271::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	ibeosdk::writeBE(os, m_objectId);
	ibeosdk::writeBE(os, m_internal);
	ibeosdk::writeBE(os, m_interfaceFlags);
	ibeosdk::writeBE(os, m_attributeFlags);
	m_unfilteredAttrib.serialize(os);
	m_filteredAttrib.serialize(os);
	ibeosdk::writeBE(os, m_reserved);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

}// namespace ibeosdk

//======================================================================
