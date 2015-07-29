//==============================================================================
/**
 * \file CarriageWay.hpp
 * \brief CarriageWay which stores CarriageWaySegments
 *
 * -----------------------------------------------------------------------------
 * \copyright &copy; 2014 Ibeo Automotive Systems GmbH, Hamburg, Germany
 *
 * \date   Oct 9, 2014
 * \author Stefan Kaufmann (stk)
 */
//==============================================================================

#include <ibeosdk/datablocks/snippets/CarriageWay.hpp>

//==============================================================================

namespace ibeosdk{
namespace lanes{

//==============================================================================

const std::string CarriageWay::VERSION = "1.0.0";

//==============================================================================

CarriageWayPtr CarriageWay::create()
{
	return CarriageWayPtr(new CarriageWay());
}

//==============================================================================

CarriageWayPtr CarriageWay::create(const UINT64& id,
                                   const UINT16& nationalId,
                                   const CarriageWayType& type,
                                   const CarriageWaySegmentMap& segments)
{
	return CarriageWayPtr(new CarriageWay(id, nationalId, type, segments));
}

//==============================================================================

CarriageWay::CarriageWay()
  : m_id(0),
    m_nationalId(0),
    m_type(CWT_UNCLASSIFIED),
    m_segments()
{
	init();
}

//==============================================================================

CarriageWay::CarriageWay(const UINT64& id,
                         const UINT16& nationalId,
                         const CarriageWayType& type,
                         const CarriageWaySegmentMap& segments)
  : m_id(id),
    m_nationalId(nationalId),
    m_type(type),
    m_segments(segments)
{
	init();
}

//=============================================================================

void CarriageWay::init()
{
	m_boundingBox = BoundingRectangle();
}

//==============================================================================

UINT16 CarriageWay::getNationalId() const
{
	return m_nationalId;
}

//=============================================================================

CarriageWayType CarriageWay::getType() const
{
	return m_type;
}

//=============================================================================

const CarriageWaySegmentMap* CarriageWay::getSegments() const
{
	return &m_segments;
}

//=============================================================================

void CarriageWay::setType(const CarriageWayType& type)
{
	m_type = type;
}

//=============================================================================

void CarriageWay::setSegments(const CarriageWaySegmentMap& segments)
{
	m_segments = segments;
}

//=============================================================================

bool CarriageWay::insert(CarriageWaySegmentPtr segment)
{
	// inserts the segment to the map with its unique id and checks if another
	// segment with the same id already exists.
	return (m_segments.insert(CarriageWaySegmentMapEntry(segment->getId(), segment))).second;
}

//=============================================================================

UINT64 CarriageWay::getId() const
{
	return m_id;
}

//=============================================================================

void CarriageWay::setId(const UINT64& id)
{
	m_id = id;
}

//=============================================================================

BoundingRectangle CarriageWay::getBoundingBox() const
{
	return m_boundingBox;
}

//=============================================================================

void CarriageWay::resolveConnections(const CarriageWayPtr& instance)
{
	// 1. Create all connections between carriageway segments, since these
	// are required for connections between lanes and lane segments
	for (CarriageWaySegmentMap::const_iterator it = m_segments.begin(); it != m_segments.end(); ++it)
	{
		// pointer to segment (the object itself has to be edited, not a copy)
		CarriageWaySegmentPtr cws = it->second;
		if (cws)
		{
			cws->setParent(instance);
			// link next element if not linked and id not 0 and the next element
			// id exists in the std::map
			if (cws->m_nextSegment.lock() == 0 && cws->m_nextId != 0 &&
					m_segments.find(cws->m_nextId) != m_segments.end() )
			{
				cws->m_nextSegment = m_segments.at(cws->m_nextId);
			}

			//link previous element if not linked and id not 0 and the next element
			// id exists in the std::map
			if (cws->m_prevSegment.lock() == 0 && cws->m_prevId != 0 &&
					m_segments.find(cws->m_prevId) != m_segments.end() )
			{
				cws->m_prevSegment = m_segments.at(cws->m_prevId);
			}
		}
	}

	// 2. Create all connections between lanes
	// now the pointers to previous and next segment between CarriageWaySegments
	// are all resolved
	for (CarriageWaySegmentMap::const_iterator it = m_segments.begin(); it != m_segments.end(); it++)
	{
		CarriageWaySegmentPtr cws = it->second;
		if (cws)
		{
			for(LaneMap::const_iterator itlane = cws->m_lanes.begin();
				itlane != cws->m_lanes.end(); ++itlane)
			{
				LanePtr lane = itlane->second;
				if (lane)
				{
					lane->setParent(cws);
					// link left neighbour
					if (lane->m_leftLane.lock() == 0 && lane->m_leftLaneId != 0 &&
							cws->m_lanes.find(lane->m_leftLaneId) != cws->m_lanes.end() )
					{
						lane->m_leftLane = cws->m_lanes.at(lane->m_leftLaneId);
					}

					// link right neighbour
					if (lane->m_rightLane.lock() == 0 && lane->m_rightLaneId != 0 &&
							cws->m_lanes.find(lane->m_rightLaneId) != cws->m_lanes.end())
					{
						lane->m_rightLane = (cws->m_lanes.at(lane->m_rightLaneId));
					}

					// link next lane (always in next CarriageWaySegment)
					if (lane->m_nextLane.lock() == 0 && lane->m_nextLaneId != 0 &&
						cws->m_nextSegment.lock() != 0 &&
						cws->m_nextSegment.lock()->m_lanes.find(lane->m_nextLaneId) !=
						cws->m_nextSegment.lock()->m_lanes.end() )
					{
						lane->m_nextLane = (cws->m_nextSegment.lock()->m_lanes.at(lane->m_nextLaneId));
					}

					// link previous lane (always in previous CarriageWaySegment)
					if (lane->m_prevLane.lock() == 0 && lane->m_prevLaneId != 0 &&
						cws->m_prevSegment.lock() != 0 &&
						cws->m_prevSegment.lock()->m_lanes.find(lane->m_prevLaneId) !=
						cws->m_prevSegment.lock()->m_lanes.end() )
					{
						lane->m_prevLane = (cws->m_prevSegment.lock()->m_lanes.at(lane->m_prevLaneId));
					}
				}
			}
		}
	}

	// 3. Create all connections between LaneSegments
	// now the pointers to previous and next lane are all resolved
	for (CarriageWaySegmentMap::const_iterator it = m_segments.begin(); it != m_segments.end(); ++it) {
		CarriageWaySegmentPtr cws = it->second;
		if (cws) {
			for(LaneMap::const_iterator itlane = cws->m_lanes.begin();
				itlane != cws->m_lanes.end(); ++itlane)
			{
				LanePtr lane = itlane->second;
				if (lane)
				{
					for (LaneSegmentMap::const_iterator itLaneSeg = lane->m_segments.begin();
						itLaneSeg != lane->m_segments.end(); ++itLaneSeg)
					{
						LaneSegmentPtr laneSeg = itLaneSeg->second;
						if (laneSeg)
						{
							laneSeg->setParent(lane);
							// link next segment if in same CarriageWaySegment
							if (!laneSeg->m_nextInNewSeg)
							{
								if(laneSeg->m_nextSegment.lock() == 0 && laneSeg->m_nextId != 0 &&
									lane->m_segments.find(laneSeg->m_nextId) != lane->m_segments.end()
								)
								{
									laneSeg->m_nextSegment = (lane->m_segments.at(laneSeg->m_nextId));
								}
							}

							// if in new CarriageWaySegment
							else
							{
								if(laneSeg->m_nextSegment.lock() == 0 && laneSeg->m_nextId != 0 &&
									lane->m_nextLane.lock() != 0 &&
									lane->m_nextLane.lock()->m_segments.find(laneSeg->m_nextId) != lane->m_nextLane.lock()->m_segments.end())
								{
									laneSeg->m_nextSegment = (lane->m_nextLane.lock()->m_segments.at(laneSeg->m_nextId));
								}
							}

							// link previous segment if in same CarriageWaySegment
							if (!laneSeg->m_prevInNewSeg)
							{
								if(laneSeg->m_prevSegment.lock() == 0 && laneSeg->m_prevId != 0 &&
									lane->m_segments.find(laneSeg->m_prevId) != lane->m_segments.end())
								{
									laneSeg->m_prevSegment = (lane->m_segments.at(laneSeg->m_prevId));
								}
							}

							// if in new CarriageWaySegment
							else
							{
								if(laneSeg->m_prevSegment.lock() == 0 && laneSeg->m_prevId != 0 &&
									lane->m_prevLane.lock() != 0 &&
									lane->m_prevLane.lock()->m_segments.find(laneSeg->m_prevId) != lane->m_prevLane.lock()->m_segments.end())
								{
									laneSeg->m_prevSegment = (lane->m_prevLane.lock()->m_segments.at(laneSeg->m_prevId));
								}
							}

							// link left
							if (laneSeg->m_leftId != 0)
							{
								LanePtr parent = laneSeg->getParent();
								if (parent->hasLeft()	&& parent->getLeft()->getSegments()->find(laneSeg->m_leftId) != parent->getLeft()->getSegments()->end())
								{
									laneSeg->m_leftSegment = parent->getLeft()->getSegments()->at(laneSeg->m_leftId);
								}
							}

							// link right
							if (laneSeg->m_rightId != 0)
							{
								LanePtr parent = laneSeg->getParent();
								if (parent->hasRight() && parent->getRight()->getSegments()->find(laneSeg->m_rightId) != parent->getRight()->getSegments()->end())
								{
									laneSeg->m_rightSegment = parent->getRight()->getSegments()->at(laneSeg->m_rightId);
								}
							}

							laneSeg->calculateLength();
							laneSeg->calculateOffsets();
							laneSeg->calculateWidth();
							laneSeg->calculateBoundingBox();

							// set bounding box for LANE from child bounds
							if (laneSeg->getBoundingBox().minLatitude < lane->getBoundingBox().minLatitude)
								lane->m_boundingBox.minLatitude = laneSeg->getBoundingBox().minLatitude;
							if (laneSeg->getBoundingBox().maxLatitude > lane->getBoundingBox().maxLatitude)
								lane->m_boundingBox.maxLatitude = laneSeg->getBoundingBox().maxLatitude;
							if (laneSeg->getBoundingBox().minLongitude < lane->getBoundingBox().minLongitude)
								lane->m_boundingBox.minLongitude = laneSeg->getBoundingBox().minLongitude;
							if (laneSeg->getBoundingBox().maxLongitude > lane->getBoundingBox().maxLongitude)
								lane->m_boundingBox.maxLongitude = laneSeg->getBoundingBox().maxLongitude;
						}
					}

					// set bounding box for CARRIAGE_WAY_SEGMENT from child bounds
					if (lane->getBoundingBox().minLatitude < cws->getBoundingBox().minLatitude)
						cws->m_boundingBox.minLatitude = lane->getBoundingBox().minLatitude;
					if (lane->getBoundingBox().maxLatitude > cws->getBoundingBox().maxLatitude)
						cws->m_boundingBox.maxLatitude = lane->getBoundingBox().maxLatitude;
					if (lane->getBoundingBox().minLongitude < cws->getBoundingBox().minLongitude)
						cws->m_boundingBox.minLongitude = lane->getBoundingBox().minLongitude;
					if (lane->getBoundingBox().maxLongitude > cws->getBoundingBox().maxLongitude)
						cws->m_boundingBox.maxLongitude = lane->getBoundingBox().maxLongitude;
				}
			}


			// set bounding box for CARRIAGE_WAY from child bounds
			if (cws->getBoundingBox().minLatitude < m_boundingBox.minLatitude)
				m_boundingBox.minLatitude = cws->getBoundingBox().minLatitude;
			if (cws->getBoundingBox().maxLatitude > m_boundingBox.maxLatitude)
				m_boundingBox.maxLatitude = cws->getBoundingBox().maxLatitude;
			if (cws->getBoundingBox().minLongitude < m_boundingBox.minLongitude)
				m_boundingBox.minLongitude = cws->getBoundingBox().minLongitude;
			if (cws->getBoundingBox().maxLongitude > m_boundingBox.maxLongitude)
				m_boundingBox.maxLongitude = cws->getBoundingBox().maxLongitude;
		} // if cws
	}
}

//==============================================================================

bool CarriageWay::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	ibeosdk::writeBE(os, m_id);                                         // 8
	ibeosdk::writeBE(os, m_nationalId);                                 // 2;
	ibeosdk::writeBE(os, static_cast<UINT8>(m_type));                   // 1
	ibeosdk::writeBE(os, static_cast<UINT64>(m_segments.size() ));      // 8

	for(CarriageWaySegmentMap::const_iterator it = m_segments.begin(); it != m_segments.end(); ++it)
		it->second->serialize(os);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//==============================================================================

std::streamsize CarriageWay::getSerializedSize() const
{
	UINT64 size = 0;

	for(CarriageWaySegmentMap::const_iterator it = m_segments.begin(); it != m_segments.end(); ++it)
		size += UINT64(it->second->getSerializedSize());

	return std::streamsize(
		8 +     // UINT64 m_id
		2 +     // UINT16 m_nationalId
		1 +     // UINT8  m_type
		8 +     // UINT64 number of segments
		size);   // size of segments
}

//==============================================================================

bool CarriageWay::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	UINT8 type;
	UINT64 nSegments;

	ibeosdk::readBE(is, m_id);
	ibeosdk::readBE(is, m_nationalId);
	ibeosdk::readBE(is, type);
	ibeosdk::readBE(is, nSegments);
	m_type = static_cast<CarriageWayType>(type);

	for(UINT64 i = 0; i < nSegments; i++)
	{
		CarriageWaySegmentPtr segment = CarriageWaySegment::create();
		segment->deserialize(is);
		insert(segment);
	}

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//==============================================================================

} // namespace lanes
} // namespace ibeo

//==============================================================================
