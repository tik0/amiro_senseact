//======================================================================
/*! \file ObjectScala2271.hpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 24, 2014
 *///-------------------------------------------------------------------
#ifndef IBEOSDK_OBJECTSCALA2271_HPP_SEEN
#define IBEOSDK_OBJECTSCALA2271_HPP_SEEN

//======================================================================

#include <ibeosdk/datablocks/snippets/UnfilteredObjectAttributes.hpp>
#include <ibeosdk/datablocks/snippets/FilteredObjectAttributes.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

class ObjectScala2271 : public ibeosdk::Snippet {
public:
	enum AttrFlags {
		NoAttributes = 0x00,
		UnfAvailable = 0x02,
		UnfContAvail = 0x01,
		FilAvailable = 0x08,
		FilContAvail = 0x04
	};
public:
	ObjectScala2271();
	virtual ~ObjectScala2271();

public:
	bool hasUnfilteredAttributes() const { return (m_attributeFlags & UnfAvailable) == UnfAvailable; }
	bool hasUnfilteredContours() const { return (m_attributeFlags & UnfContAvail) == UnfContAvail; }
	bool hasFilteredAttributes() const { return (m_attributeFlags & FilAvailable) == FilAvailable; }
	bool hasFilteredContours() const { return (m_attributeFlags & FilContAvail) == FilContAvail; }

public:
	virtual std::streamsize getSerializedSize() const;
	virtual bool deserialize(std::istream& is);
	virtual bool serialize(std::ostream& os) const;

public:
	uint32_t getObjectId() const { return m_objectId; }
	uint8_t getInterfaceFlags() const { return m_interfaceFlags; }
	uint8_t getAttributeFlags() const { return m_attributeFlags; }
	const UnfilteredObjectAttributes& getUnfilteredObjectAttributes() const { return m_unfilteredAttrib; }
	UnfilteredObjectAttributes& getUnfilteredObjectAttributes() { return m_unfilteredAttrib; }
	const FilteredObjectAttributes& getFilteredObjectAttributes() const { return m_filteredAttrib; }
	FilteredObjectAttributes& getFilteredObjectAttributes() { return m_filteredAttrib; }

public:
	void setObjectId(const uint32_t objectId) { m_objectId = objectId; }
	void setInterfaceFlags(const uint8_t interfaceFlags) { m_interfaceFlags = interfaceFlags; }
	void setAttributeFlags(const uint8_t attributeFlags) { m_attributeFlags = attributeFlags; }

protected:
	uint32_t m_objectId;
	uint8_t m_internal;
	uint8_t m_interfaceFlags;
	uint8_t m_attributeFlags;
	UnfilteredObjectAttributes m_unfilteredAttrib;
	FilteredObjectAttributes m_filteredAttrib;
	uint32_t m_reserved;
}; // ObjectScala2271

//======================================================================

}// namespace ibeosdk

//======================================================================

#endif // IBEOSDK_OBJECTSCALA2271_HPP_SEEN

//======================================================================
