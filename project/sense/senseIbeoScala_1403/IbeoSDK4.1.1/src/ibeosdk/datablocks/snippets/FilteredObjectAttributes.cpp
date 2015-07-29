//======================================================================
/*! \file FilteredObjectAttributes.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Apr 23, 2014
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/snippets/FilteredObjectAttributes.hpp>

#include <ibeosdk/ObjectBasic.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

const uint8_t DynamicFlagExistenceValues::mask     = 0x01;
const uint8_t DynamicFlagExistenceValues::init     =    0;
const uint8_t DynamicFlagExistenceValues::existing =    1;

//======================================================================

const uint8_t DynamicFlagDynState::mask             = 0x07 << 1;
const uint8_t DynamicFlagDynState::unknownDynState  =    0 << 1;
const uint8_t DynamicFlagDynState::dynamicMoving    =    1 << 1;
const uint8_t DynamicFlagDynState::dynamicStopped   =    2 << 1;
const uint8_t DynamicFlagDynState::dynamicStarted   =    3 << 1;
const uint8_t DynamicFlagDynState::aPrioryStaticObj =    4 << 1;

//======================================================================

FilteredObjectAttributes::FilteredObjectAttributes()
  : m_isValid(false),
    m_hasContourPoints(false),
    m_priority(0),

    m_objectAge(0),
    m_hiddenStatusAge(0),
    m_dynamicFlag(0),

    m_relativeTimeOfMeasure(0),
    m_positionClosestObjectPoint(),

    m_relativeVelocity(),
    m_relativeVelocitySigma(),
    m_classification(ObjectClass_Unclassified),
    m_classificationQuality(0),
    m_classificationAge(0),

    m_reserved(0),
    m_objectBoxSize(),
    m_objectBoxSizeSigma(),
    m_objectBoxOrientation(0),
    m_objectBoxOrientationSigma(0),
    m_objectBoxHeight(0),
    m_referencePointLocation(RPL_Unknown),
    m_referencePointCoord(),
    m_referencePointCoordSigma(),
    m_referencePointPositionCorrCoeff(0),
    m_existenceProbaility(0),

    m_absoluteVelocity(),
    m_absoluteVelocitySigma(),
    m_velocityCorrCoeff(0),
    m_acceleration(),
    m_accelerationSigma(),
    m_accelerationCorrCoeff(0),
    m_yawRate(0),
    m_yawRateSigma(0),

    m_possibleNbOfContourPoints(0),
    m_contourPoints()
{}

//======================================================================

FilteredObjectAttributes::~FilteredObjectAttributes() {}

//======================================================================

std::streamsize FilteredObjectAttributes::getSerializedSize() const
{
	if (!m_isValid)
		return 0;

	return std::streamsize(sizeof(uint8_t))
	     + 2*std::streamsize(sizeof(uint16_t))
	     + std::streamsize(sizeof(uint8_t))

	     + std::streamsize(sizeof(uint16_t))  // rel t of measure
	     + Point2d::getSerializedSize()

	     + Point2d::getSerializedSize()
	     + PointSigma2d::getSerializedSize()
	     + 2*std::streamsize(sizeof(uint8_t))
	     + std::streamsize(sizeof(uint16_t))  // class age

	     + std::streamsize(sizeof(uint16_t))
	     + 2*PointSigma2d::getSerializedSize() // objBox sz/sigma
	     + 2*std::streamsize(sizeof(uint16_t))
	     + 2*std::streamsize(sizeof(uint8_t))  // ... refPointLoc
	     + Point2d::getSerializedSize()
	     + PointSigma2d::getSerializedSize()
	     + std::streamsize(sizeof(int16_t))
	     + std::streamsize(sizeof(uint8_t)) // exist Prob

	     + Point2d::getSerializedSize()
	     + PointSigma2d::getSerializedSize()
	     + std::streamsize(sizeof(int16_t))
	     + Point2d::getSerializedSize()
	     + PointSigma2d::getSerializedSize()
	     + std::streamsize(sizeof(int16_t)) // accCorrCoeff
	     + 2*std::streamsize(sizeof(uint16_t)) // yawRate, yawRateSigma

	     + std::streamsize(sizeof(uint8_t)) // nb of points
	     + (m_hasContourPoints ? m_possibleNbOfContourPoints * ContourPoint::getSerializedSize_static()
	                           : 0);
}

//======================================================================

bool FilteredObjectAttributes::deserialize(std::istream& is)
{
	if (!m_isValid)
		return true;

	const std::istream::pos_type startPos = is.tellg();

	ibeosdk::readBE(is, m_priority);

	ibeosdk::readBE(is, m_objectAge);
	ibeosdk::readBE(is, m_hiddenStatusAge);
	ibeosdk::readBE(is, m_dynamicFlag);

	ibeosdk::readBE(is, m_relativeTimeOfMeasure);
	ibeosdk::readBE(is, m_positionClosestObjectPoint);

	ibeosdk::readBE(is, m_relativeVelocity);
	ibeosdk::readBE(is, m_relativeVelocitySigma);
	ibeosdk::readBE(is, m_classification);
	ibeosdk::readBE(is, m_classificationQuality);
	ibeosdk::readBE(is, m_classificationAge);

	ibeosdk::readBE(is, m_reserved);
	ibeosdk::readBE(is, m_objectBoxSize);
	ibeosdk::readBE(is, m_objectBoxSizeSigma);
	ibeosdk::readBE(is, m_objectBoxOrientation);
	ibeosdk::readBE(is, m_objectBoxOrientationSigma);
	ibeosdk::readBE(is, m_objectBoxHeight);
	ibeosdk::readBE<8>(is, m_referencePointLocation);
	ibeosdk::readBE(is, m_referencePointCoord);
	ibeosdk::readBE(is, m_referencePointCoordSigma);
	ibeosdk::readBE(is, m_referencePointPositionCorrCoeff);
	ibeosdk::readBE(is, m_existenceProbaility);

	ibeosdk::readBE(is, m_absoluteVelocity);
	ibeosdk::readBE(is, m_absoluteVelocitySigma);
	ibeosdk::readBE(is, m_velocityCorrCoeff);
	ibeosdk::readBE(is, m_acceleration);
	ibeosdk::readBE(is, m_accelerationSigma);
	ibeosdk::readBE(is, m_accelerationCorrCoeff);
	ibeosdk::readBE(is, m_yawRate);
	ibeosdk::readBE(is, m_yawRateSigma);


	ibeosdk::readBE(is, m_possibleNbOfContourPoints);
	// only if this FilteredObjectAttributes object is dedicated to
	// have contour points, nbOfContourPoints presents the number
	// of contour points that have been attached.
	// Otherwise nbOfContourPoints is just the number of contour
	// points that would have been written in case it was allowed to.
	if (m_hasContourPoints) {
		m_contourPoints.resize(m_possibleNbOfContourPoints);
		std::vector<ContourPoint>::iterator cpIter = m_contourPoints.begin();
		for (; cpIter != m_contourPoints.end(); ++cpIter) {
			cpIter->deserialize(is);
		}
	}
	else {
		m_contourPoints.clear();
	}

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());

}

//======================================================================

bool FilteredObjectAttributes::serialize(std::ostream& os) const
{
	if (!m_isValid)
		return true;

	const std::istream::pos_type startPos = os.tellp();

	ibeosdk::writeBE(os, m_priority);

	ibeosdk::writeBE(os, m_objectAge);
	ibeosdk::writeBE(os, m_hiddenStatusAge);
	ibeosdk::writeBE(os, m_dynamicFlag);

	ibeosdk::writeBE(os, m_relativeTimeOfMeasure);
	ibeosdk::writeBE(os, m_positionClosestObjectPoint);

	ibeosdk::writeBE(os, m_relativeVelocity);
	ibeosdk::writeBE(os, m_relativeVelocitySigma);
	ibeosdk::writeBE(os, m_classification);
	ibeosdk::writeBE(os, m_classificationQuality);
	ibeosdk::writeBE(os, m_classificationAge);

	ibeosdk::writeBE(os, m_reserved);
	ibeosdk::writeBE(os, m_objectBoxSize);
	ibeosdk::writeBE(os, m_objectBoxSizeSigma);
	ibeosdk::writeBE(os, m_objectBoxOrientation);
	ibeosdk::writeBE(os, m_objectBoxOrientationSigma);
	ibeosdk::writeBE(os, m_objectBoxHeight);
	ibeosdk::writeBE<8>(os, m_referencePointLocation);
	ibeosdk::writeBE(os, m_referencePointCoord);
	ibeosdk::writeBE(os, m_referencePointCoordSigma);
	ibeosdk::writeBE(os, m_referencePointPositionCorrCoeff);
	ibeosdk::writeBE(os, m_existenceProbaility);

	ibeosdk::writeBE(os, m_absoluteVelocity);
	ibeosdk::writeBE(os, m_absoluteVelocitySigma);
	ibeosdk::writeBE(os, m_velocityCorrCoeff);
	ibeosdk::writeBE(os, m_acceleration);
	ibeosdk::writeBE(os, m_accelerationSigma);
	ibeosdk::writeBE(os, m_accelerationCorrCoeff);
	ibeosdk::writeBE(os, m_yawRate);
	ibeosdk::writeBE(os, m_yawRateSigma);

	if (!m_contourPoints.empty())
		m_possibleNbOfContourPoints = uint8_t(m_contourPoints.size());

	ibeosdk::writeBE(os, m_possibleNbOfContourPoints);

	// only if this UnfilteredObjectAttributes object is dedicated to
	// have contour points, the contour points will be stored.
	if (m_hasContourPoints) {
		std::vector<ContourPoint>::const_iterator cpIter = m_contourPoints.begin();
		for (; cpIter != m_contourPoints.end(); ++cpIter) {
			cpIter->serialize(os);
		}
	}

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());

}

//======================================================================

}// namespace ibeosdk

//======================================================================
