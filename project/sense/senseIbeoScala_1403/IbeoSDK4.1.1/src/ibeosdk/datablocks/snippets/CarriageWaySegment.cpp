//==============================================================================
/**
 * \file CarriageWaySegment.cpp
 * \brief CarriageWaySegment which has a constant number of lanes
 *
 * -----------------------------------------------------------------------------
 * \copyright &copy; 2014 Ibeo Automotive Systems GmbH, Hamburg, Germany
 *
 * \date   Oct 9, 2014
 * \author Stefan Kaufmann (stk)
 */
//==============================================================================

#include <ibeosdk/datablocks/snippets/CarriageWaySegment.hpp>
#include <ibeosdk/datablocks/snippets/CarriageWay.hpp>

//======================================================================

namespace ibeosdk{
namespace lanes{

//==============================================================================

std::string const CarriageWaySegment::VERSION = "1.0.0";

//==============================================================================

CarriageWaySegmentPtr CarriageWaySegment::create()
{
	return CarriageWaySegmentPtr(new CarriageWaySegment());
}

//==============================================================================

CarriageWaySegmentPtr CarriageWaySegment::create(CarriageWayPtr parent)
{
	return CarriageWaySegmentPtr(new CarriageWaySegment(parent));
}

//==============================================================================

CarriageWaySegmentPtr CarriageWaySegment::create(const UINT64& id,
                                                 const UINT8& nLanes,
                                                 CarriageWayPtr parent)
{
	return CarriageWaySegmentPtr(new CarriageWaySegment(id, nLanes, parent) );
}

//==============================================================================

CarriageWaySegmentPtr CarriageWaySegment::create(const UINT64& id,
                                                 const UINT8& nLanes,
                                                 CarriageWayPtr parent,
                                                 const UINT64 nextSegmentID,
                                                 const UINT64 previousSegmentID)
{
	return CarriageWaySegmentPtr(new CarriageWaySegment(id, nLanes, parent,
		nextSegmentID, previousSegmentID));
}

//==============================================================================

CarriageWaySegment::CarriageWaySegment(const UINT64& id,
                                       const UINT8& nLanes,
                                       CarriageWayPtr parent)
  : m_id(id),
    m_nLanes(nLanes),
    m_parent(parent)
{
	init();
}

//==============================================================================

CarriageWaySegment::CarriageWaySegment(const UINT64& id,
                                       const UINT8& nLanes,
                                       CarriageWayPtr parent,
                                       const UINT64 nextSegmentID,
                                       const UINT64 previousSegmentID)
  : m_id(id),
    m_nLanes(nLanes),
    m_parent(parent),
    m_nextId(nextSegmentID),
    m_prevId(previousSegmentID)
{
	init();
}

//==============================================================================

CarriageWaySegment::CarriageWaySegment(CarriageWayPtr parent)
  : m_id(0),
    m_nLanes(0),
    m_parent(parent),
    m_nextId(0),
    m_prevId(0)
{
	init();
}

//==============================================================================

CarriageWaySegment::CarriageWaySegment()
  : m_id(0),
    m_nLanes(0),
    m_parent(CarriageWayPtr()),
    m_nextId(0),
    m_prevId(0)
{
	init();
}

//==============================================================================

void CarriageWaySegment::init()
{
	m_lanes = LaneMap();
	m_boundingBox = BoundingRectangle();
	m_nextSegment = CarriageWaySegmentPtr();
	m_prevSegment = CarriageWaySegmentPtr();
}

//==============================================================================

const LaneMap* CarriageWaySegment::getLanes() const
{
	return &m_lanes;
}

//=============================================================================

UINT8 CarriageWaySegment::getNumberOfLanes() const
{
	return UINT8(m_lanes.size());
}

//=============================================================================

const CarriageWaySegmentPtr CarriageWaySegment::getNext() const
{
	return m_nextSegment.lock();
}

//=============================================================================

void CarriageWaySegment::setNext(const CarriageWaySegmentPtr& next)
{
	m_nextSegment = next;
}

//=============================================================================

const CarriageWaySegmentPtr CarriageWaySegment::getPrevious() const
{
	return m_prevSegment.lock();
}

//=============================================================================

void CarriageWaySegment::setPrevious(const CarriageWaySegmentPtr& previous)
{
	m_prevSegment = previous;
}

//=============================================================================

void CarriageWaySegment::setParent(const CarriageWayPtr& parent)
{
	m_parent = parent;
}

//=============================================================================

const CarriageWayPtr CarriageWaySegment::getParent() const
{
	return m_parent.lock();
}

//=============================================================================

UINT64 CarriageWaySegment::getId() const
{
	return m_id;
}

//=============================================================================

bool CarriageWaySegment::insert(LanePtr lane)
{
	// inserts the lane to the map with its unique id and checks if another
	// lane with the same id already exists.
	return (m_lanes.insert(LaneMapEntry(lane->getId(), lane))).second;
}

//=============================================================================

bool CarriageWaySegment::hasNext() const
{
	return m_nextSegment.lock() == 0 ? false : true;
}

//=============================================================================

bool CarriageWaySegment::hasPrevious() const
{
	return m_prevSegment.lock() == 0 ? false : true;
}

//=============================================================================

BoundingRectangle CarriageWaySegment::getBoundingBox() const
{
	return m_boundingBox;
}

//=============================================================================

std::streamsize CarriageWaySegment::getSerializedSize() const
{
	std::streamsize size = 0;

	for(LaneMap::const_iterator it = m_lanes.begin(); it != m_lanes.end(); ++it)
		size += it->second->getSerializedSize();

	return
				8 +                                // UINT64 m_id
				1 +                                // UINT8  m_lanes
				8 +                                // UINT64 m_nextId
				8 +                                // UINT64 m_prevId
				size;
}

//=============================================================================

bool CarriageWaySegment::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	UINT64 u0 = 0;

	ibeosdk::writeBE(os, getId());                                     // 8
	ibeosdk::writeBE(os, getNumberOfLanes());                          // 1;
	ibeosdk::writeBE(os, hasNext() ? getNext()->getId() : u0);         // 8;
	ibeosdk::writeBE(os, hasPrevious() ? getPrevious()->getId() : u0); // 8;

	for(LaneMap::const_iterator it = m_lanes.begin(); it != m_lanes.end(); ++it)
		it->second->serialize(os);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//=============================================================================

bool CarriageWaySegment::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	ibeosdk::readBE(is, m_id);
	ibeosdk::readBE(is, m_nLanes);
	ibeosdk::readBE(is, m_nextId);
	ibeosdk::readBE(is, m_prevId);

	for(UINT16 i = 0; i < m_nLanes; ++i)
	{
		LanePtr lane = Lane::create();
		lane->deserialize(is);
		insert(lane);
	}

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//==============================================================================

void CarriageWaySegment::setLanes(const LaneMap& lanes)
{
	m_lanes = lanes;
}

//==============================================================================

} // namespace lanes
} // namespace ibeo
