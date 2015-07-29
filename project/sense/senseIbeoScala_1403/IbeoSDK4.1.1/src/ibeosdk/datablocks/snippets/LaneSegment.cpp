//==============================================================================
/**
 * \file LaneSegment.cpp
 * \brief Segment of a lane
 *
 * -----------------------------------------------------------------------------
 * \copyright &copy; 2014 Ibeo Automotive Systems GmbH, Hamburg, Germany
 *
 * \date   Oct 9, 2014
 * \author Stefan Kaufmann (stk)
 */
//==============================================================================

#include <ibeosdk/datablocks/snippets/LaneSegment.hpp>
#include <ibeosdk/Line2dFloat.hpp>
#include <ibeosdk/TransformationMatrix2dFloat.hpp>

#include <boost/shared_ptr.hpp>
#include <vector>

namespace ibeosdk{
namespace lanes{

//==============================================================================

LaneSegmentPtr LaneSegment::create()
{
	return LaneSegmentPtr(new LaneSegment());
}

//==============================================================================

LaneSegmentPtr LaneSegment::create(LanePtr parent)
{
	return LaneSegmentPtr(new LaneSegment(parent));
}

//==============================================================================

LaneSegmentPtr LaneSegment::create(const LaneSupportPoint& laneStart)
{
	return LaneSegmentPtr(new LaneSegment(laneStart));
}

//==============================================================================

LaneSegmentPtr LaneSegment::create(const UINT64& id, const LaneMarkingType& markingLeft,
	const LaneMarkingType& markingRight, const BorderType& borderLeft, const BorderType& borderRight,
	const LaneSupportPoint& laneStart, const UINT64& nextId, const UINT64& prevId, const UINT64& leftId,
	const UINT64& rightId, const bool& nextInNewSeg, const bool& prevInNewSeg, LanePtr parent)
{
	return LaneSegmentPtr(new LaneSegment(id, markingLeft, markingRight, borderLeft, borderRight,
		laneStart, nextId, prevId, leftId, rightId, nextInNewSeg, prevInNewSeg, parent
	));
}

//==============================================================================

LaneSegment::LaneSegment(const UINT64& id,
						 const LaneMarkingType& markingLeft,
						 const LaneMarkingType& markingRight,
						 const BorderType& borderLeft,
						 const BorderType& borderRight,
						 const LaneSupportPoint& laneStart,
						 const UINT64& nextId,
						 const UINT64& prevId,
						 const UINT64& leftId,
						 const UINT64& rightId,
						 const bool& nextInNewSeg,
						 const bool& prevInNewSeg,
						 LanePtr parent)
	:
	m_id(id),
	m_markingLeft(markingLeft),
	m_markingRight(markingRight),
	m_borderLeft(borderLeft),
	m_borderRight(borderRight),
	m_start(laneStart),
	m_nextId(nextId),
	m_prevId(prevId),
	m_leftId(leftId),
	m_rightId(rightId),
	m_nextInNewSeg(nextInNewSeg),
	m_prevInNewSeg(prevInNewSeg),
	m_parent(parent)
{
	init();
}

//==============================================================================

LaneSegment::LaneSegment(const LanePtr& parent) : m_parent(parent)
{
	m_id = 0;
	m_markingLeft = LMT_UNCLASSIFIED;
	m_markingRight = LMT_UNCLASSIFIED;
	m_borderLeft = BT_UNCLASSIFIED;
	m_borderRight = BT_UNCLASSIFIED;
	m_start = LaneSupportPoint();
	m_nextId = 0;
	m_prevId = 0;
	m_nextInNewSeg = false;
	m_prevInNewSeg = false;
	init();
}

//==============================================================================

LaneSegment::LaneSegment(const LaneSupportPoint& laneStart)
{
	m_id = 0;
	m_markingLeft = LMT_UNCLASSIFIED;
	m_markingRight = LMT_UNCLASSIFIED;
	m_borderLeft = BT_UNCLASSIFIED;
	m_borderRight = BT_UNCLASSIFIED;
	m_start = laneStart;
	m_nextId = 0;
	m_prevId = 0;
	m_nextInNewSeg = false;
	m_prevInNewSeg = false;
	m_parent = LanePtr();
	init();
}

//==============================================================================

LaneSegment::LaneSegment()
{
	m_id = 0;
	m_markingLeft = LMT_UNCLASSIFIED;
	m_markingRight = LMT_UNCLASSIFIED;
	m_borderLeft = BT_UNCLASSIFIED;
	m_borderRight = BT_UNCLASSIFIED;
	m_start = LaneSupportPoint();
	m_nextId = 0;
	m_prevId = 0;
	m_nextInNewSeg = false;
	m_prevInNewSeg = false;
	m_parent = LanePtr();
	init();
}

//==============================================================================

void LaneSegment::init()
{
	m_nextSegment = LaneSegmentPtr();
	m_prevSegment = LaneSegmentPtr();
	m_boundingBox = BoundingRectangle();
}

//==============================================================================

std::string const LaneSegment::VERSION = "1.0.0";

//==============================================================================

UINT64 LaneSegment::getId() const
{
	return m_id;
}

//==============================================================================

void LaneSegment::setId(const UINT64& id)
{
	m_id = id;
}

//==============================================================================

LaneSegmentPtr LaneSegment::getNext() const
{
	return m_nextSegment.lock();
}

//==============================================================================

void LaneSegment::setNext(const LaneSegmentPtr& next)
{
	m_nextSegment = next;
}

//==============================================================================

bool LaneSegment::hasNext() const
{
	return static_cast<bool>(m_nextSegment.lock());
}

//==============================================================================

LaneSegmentPtr LaneSegment::getPrevious() const
{
	return m_prevSegment.lock();
}

//==============================================================================

void LaneSegment::setPrevious(const LaneSegmentPtr& previous)
{
	m_prevSegment = previous;
}

//==============================================================================

void LaneSegment::setLeft(const LaneSegmentPtr& left)
{
	m_leftSegment = left;
}

//==============================================================================

LaneSegmentPtr LaneSegment::getLeft() const
{
	return m_leftSegment.lock();
}

//==============================================================================

void LaneSegment::setRight(const LaneSegmentPtr& right)
{
	m_rightSegment = right;
}

//==============================================================================

LaneSegmentPtr LaneSegment::getRight() const
{
	return m_rightSegment.lock();
}

//==============================================================================

bool LaneSegment::hasPrevious() const
{
	return static_cast<bool>(m_prevSegment.lock());
}

//==============================================================================

bool LaneSegment::hasLeft() const
{
	return static_cast<bool>(m_leftSegment.lock());
}

//==============================================================================

bool LaneSegment::hasRight() const
{
	return static_cast<bool>(m_rightSegment.lock());
}

//==============================================================================

LanePtr LaneSegment::getParent() const
{
		return m_parent.lock();
}

//==============================================================================

void LaneSegment::setParent(const LanePtr& parent)
{
	m_parent = parent;
}

//==============================================================================

LaneMarkingType LaneSegment::getLeftMarkingType() const
{
	return m_markingLeft;
}

//==============================================================================

LaneMarkingType LaneSegment::getRightMarkingType() const
{
	return m_markingRight;
}

//==============================================================================

BorderType LaneSegment::getLeftBorderType() const
{
	return m_borderLeft;
}

//==============================================================================

BorderType LaneSegment::getRightBorderType() const
{
	return m_borderRight;
}

//==============================================================================

LaneSupportPoint LaneSegment::getStartPoint() const
{
	return m_start;
}

//==============================================================================

BoundingRectangle LaneSegment::getBoundingBox() const
{
	return m_boundingBox;
}

//==============================================================================

float LaneSegment::getLength() const
{
	return m_length;
}

//==============================================================================

void LaneSegment::calculateLength()
{
	if (hasNext())
	{
		double x, y;
		m_nextSegment.lock()->getStartPoint().getPoint().transformToTangentialPlane(m_start.getPoint(), &x, &y);
		m_length = float(sqrt(x*x + y*y));
	}
}

//==============================================================================

void LaneSegment::calculateWidth()
{
	if (hasNext())
	{
		Line2dFloat left(m_start.getLeftOffset(), getEndOffsetLeft());
		Line2dFloat right(m_start.getRightOffset(), getEndOffsetRight());

		Line2dFloat startCutLine(Point2dFloat(0, 100), Point2dFloat(0, -100));
		Line2dFloat endCutLine(Point2dFloat(getLength(), 100), Point2dFloat(getLength(), -100));

		Point2dFloat cls, crs, cle, cre;
		startCutLine.isIntersecting(left, &cls);
		startCutLine.isIntersecting(right, &crs);
		endCutLine.isIntersecting(left, &cle);
		endCutLine.isIntersecting(right, &cre);

		m_startWidth = (cls - crs).dist();
		m_endWidth = (cle-cre).dist();
	}
}

//==============================================================================

float LaneSegment::getWidth(const float position) const
{
	if (position < getLength())
	{
		if (getLength() > 0)
		{
			return (m_endWidth - m_startWidth)/getLength() * position + m_startWidth;
		}
		else return 0.0f;
	}

	else if (hasNext() )
		return m_nextSegment.lock()->getWidth(position - getLength());
	else
		return 0.0f;
}

//==============================================================================

void LaneSegment::calculateBoundingBox()
{
	if (hasNext())
	{
		// create bounding box
		PositionWgs84 dummy;
		std::vector<PositionWgs84> points;
		dummy.transformFromTangentialPlaneWithHeading(m_start.getLeftOffset().getX(), m_start.getLeftOffset().getY(), m_start.getPoint() ); points.push_back(dummy);
		dummy.transformFromTangentialPlaneWithHeading(m_start.getRightOffset().getX(),  m_start.getRightOffset().getY(), m_start.getPoint() ); points.push_back(dummy);
		dummy.transformFromTangentialPlaneWithHeading(getEndOffsetLeft().getX(), getEndOffsetLeft().getY() , m_start.getPoint() ); points.push_back(dummy);
		dummy.transformFromTangentialPlaneWithHeading(getEndOffsetRight().getX(), getEndOffsetRight().getY(),  m_start.getPoint() ); points.push_back(dummy);

		bool initialized = false;
		for (std::vector<PositionWgs84>::iterator it = points.begin(); it != points.end(); it++)
		{
			if ((*it).getLatitudeInDeg() < m_boundingBox.minLatitude || !initialized)
				m_boundingBox.minLatitude = (*it).getLatitudeInDeg();
			if ((*it).getLatitudeInDeg() > m_boundingBox.maxLatitude || !initialized)
				m_boundingBox.maxLatitude = (*it).getLatitudeInDeg();
			if ((*it).getLongitudeInDeg() < m_boundingBox.minLongitude || !initialized)
				m_boundingBox.minLongitude = (*it).getLongitudeInDeg();
			if ((*it).getLongitudeInDeg() > m_boundingBox.maxLongitude || !initialized)
				m_boundingBox.maxLongitude = (*it).getLongitudeInDeg();

			initialized = true;
		}
	}
}

//==============================================================================

void LaneSegment::calculateOffsets()
{
	if (hasNext())
	{
		LaneSupportPoint endSupport = m_nextSegment.lock()->getStartPoint();

		double x, y;
		endSupport.getPoint().transformToTangentialPlaneWithHeading(m_start.getPoint(), &x, &y);
		m_endOffset =  Point2dFloat(float(x), float(y));

		TransformationMatrix2dFloat start2World(float(m_start.getPoint().getCourseAngleInRad()));
		TransformationMatrix2dFloat end2World(float(endSupport.getPoint().getCourseAngleInRad()));
		TransformationMatrix2dFloat end2start( (start2World.inverted() * end2World).getRotation(), m_endOffset);

		m_endOffsetLeft = end2start * endSupport.getLeftOffset();
		m_endOffsetRight = end2start * endSupport.getRightOffset();
	}
}

//==============================================================================

Point2dFloat LaneSegment::getEndOffset() const
{
	return m_endOffset;
}

//==============================================================================

Point2dFloat LaneSegment::getEndOffsetLeft() const
{
	return m_endOffsetLeft;
}

//==============================================================================

Point2dFloat LaneSegment::getEndOffsetRight() const
{
	return m_endOffsetRight;
}

//==============================================================================

Point2dFloat LaneSegment::getStartOffsetLeft() const
{
	return m_start.getLeftOffset();
}

//==============================================================================

Point2dFloat LaneSegment::getStartOffsetRight() const
{
	return m_start.getRightOffset();
}

//==============================================================================

std::streamsize LaneSegment::getSerializedSize() const
{
	return
			8 +                 // UINT64    m_id
			8 +                 // UINT64    m_nextId
			8 +                 // UINT64    m_prevId
			8 +                 // UINT64    m_leftId
			8 +                 // UINT64    m_rightId
			1 +                 // UINT8     m_markingLeft
			1 +                 // UINT8     m_markingRight
			1 +                 // UINT8     m_borderLeft
			1 +                 // UINT8     m_borderRight
			1 +                 // UINT8     m_nextInNewSeg
			1 +                 // UINT8     m_prevInNewSeg
			m_start.getSerializedSize(); 		// unknown
}

//==============================================================================

bool LaneSegment::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	UINT64 u0 = 0;

	ibeosdk::writeBE(os, getId());                                                                         // 8
	ibeosdk::writeBE(os, hasNext() ? getNext()->getId() : u0);                                             // 8
	ibeosdk::writeBE(os, hasPrevious() ? getPrevious()->getId() : u0);                                     // 8
	ibeosdk::writeBE(os, hasLeft() ? getLeft()->getId() : u0);                                             // 8
	ibeosdk::writeBE(os, hasRight() ? getRight()->getId() : u0);                                           // 8
	ibeosdk::writeBE(os, static_cast<UINT8>(m_markingLeft) );                                              // 1;
	ibeosdk::writeBE(os, static_cast<UINT8>(m_markingRight) );                                             // 1
	ibeosdk::writeBE(os, static_cast<UINT8>(m_borderLeft) );                                               // 1;
	ibeosdk::writeBE(os, static_cast<UINT8>(m_borderRight) );                                              // 1;
	ibeosdk::writeBE(os, hasNext() && getParent() == getNext()->getParent() ? false : true);               // 1;
	ibeosdk::writeBE(os, hasPrevious() && getParent() == getPrevious()->getParent() ? false : true);       // 1;

	// write LaneSupportPoints
	m_start.serialize(os);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//==============================================================================

bool LaneSegment::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	UINT8 lmtl, lmtr, btl, btr;

	ibeosdk::readBE(is, m_id);
	ibeosdk::readBE(is, m_nextId);
	ibeosdk::readBE(is, m_prevId);
	ibeosdk::readBE(is, m_leftId);
	ibeosdk::readBE(is, m_rightId);
	ibeosdk::readBE(is, lmtl);
	ibeosdk::readBE(is, lmtr);
	ibeosdk::readBE(is, btl);
	ibeosdk::readBE(is, btr);
	ibeosdk::readBE(is, m_nextInNewSeg);
	ibeosdk::readBE(is, m_prevInNewSeg);

	m_markingLeft  = static_cast<LaneMarkingType>( lmtl );
	m_markingRight = static_cast<LaneMarkingType>( lmtr );
	m_borderLeft   = static_cast<BorderType>( btl );
	m_borderRight  = static_cast<BorderType>( btr );

	m_start.deserialize(is);

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//==============================================================================

PositionWgs84 LaneSegment::getEndGps()
{
	PositionWgs84 out;
	out.transformFromTangentialPlaneWithHeading(getLength(), .0f, m_start.getPoint());
	return out;
}

//==============================================================================

} // namespace lanes
} // namespace ibeo
