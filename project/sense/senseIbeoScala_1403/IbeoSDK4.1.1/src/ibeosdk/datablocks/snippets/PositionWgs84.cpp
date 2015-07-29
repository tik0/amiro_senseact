//======================================================================
/*! \file PositionWgs84.cpp
 *
 * \copydoc Copyright
 * \author Jan Christian Dittmer (jcd)
 * \date Jan 24, 2014
 *///-------------------------------------------------------------------
//======================================================================

#include <ibeosdk/datablocks/snippets/PositionWgs84.hpp>
#include <ibeosdk/DataBlockBufferProvider.hpp>
#include <ibeosdk/io.hpp>

//======================================================================

namespace ibeosdk {

//======================================================================

template<>
inline
void readBE<PositionWgs84::SourceType>(std::istream& is, PositionWgs84::SourceType& value)
{
	uint16_t tmp;
	readBE(is, tmp);
	value = PositionWgs84::SourceType(tmp);
}

//======================================================================

template<>
inline
void writeBE<PositionWgs84::SourceType>(std::ostream& os, const PositionWgs84::SourceType& value)
{
	const uint16_t tmp = value;
	writeBE(os, tmp);
}

//======================================================================

PositionWgs84::PositionWgs84()
{}


//======================================================================

PositionWgs84::~PositionWgs84() {}

//======================================================================

void PositionWgs84::transformToTangentialPlane(const PositionWgs84& origin, double* pdX, double* pdY) const
{
	const double dGps_SemiMajorAxis	= 6378137.0;
	const double dGps_SemiMinorAxis	= 6356752.3142;
	const double dGps_Flatness		= (dGps_SemiMajorAxis - dGps_SemiMinorAxis) / dGps_SemiMajorAxis;
	const double dGps_Eccentricity2	= dGps_Flatness * (2.0 - dGps_Flatness);

	const double dLat = getLatitudeInRad();		// (GPS_PI / 180.0);
	const double dLong = getLongitudeInRad();	// (GPS_PI / 180.0);

	const double dOriginLat = origin.getLatitudeInRad();	// (GPS_PI / 180.0);
	const double dOriginLong = origin.getLongitudeInRad();	// (GPS_PI / 180.0);

	double dX_ECEF = 0.0;
	double dY_ECEF = 0.0;
	double dZ_ECEF = 0.0;
	double dHeight = 0.0;

	// WGS --> ECEF
	double dN;
	dN = dGps_SemiMajorAxis / (sqrt(1.0 - (dGps_Eccentricity2 * sin(dLat) * sin(dLat))));
	dX_ECEF = (dN + dHeight) * cos(dLat) * cos(dLong);
	dY_ECEF = (dN + dHeight) * cos(dLat) * sin(dLong);
	dZ_ECEF = ((dN * (1.0 - (dGps_Eccentricity2))) + dHeight) * sin(dLat);

	// ECEF --> TP
	double dX_0_ECEF;
	double dY_0_ECEF;
	double dZ_0_ECEF;
	double dX_TP;
	double dY_TP;
	double dZ_TP;

	// orgin of the Tangentplane in ECEF-coordinates
	dN = dGps_SemiMajorAxis / (sqrt(1.0 - (dGps_Eccentricity2 * sin(dOriginLat) * sin(dOriginLat))));
	dX_0_ECEF = (dN + dHeight) * cos(dOriginLat) * cos(dOriginLong);
	dY_0_ECEF = (dN + dHeight) * cos(dOriginLat) * sin(dOriginLong);
	dZ_0_ECEF = ((dN * (1.0 - (dGps_Eccentricity2))) + dHeight) * sin(dOriginLat);


	// see "Geographic Coordiante Transformation and
	//      Landmark Navigation" (T.Weiss)
	dX_TP = - sin(dOriginLat) * cos(dOriginLong) * (dX_ECEF - dX_0_ECEF)
	- sin(dOriginLat) * sin(dOriginLong) * (dY_ECEF - dY_0_ECEF)
	+ cos(dOriginLat) * (dZ_ECEF - dZ_0_ECEF);

	dY_TP =   sin(dOriginLong) * (dX_ECEF - dX_0_ECEF)
	- cos(dOriginLong) * (dY_ECEF - dY_0_ECEF);

	dZ_TP =   cos(dOriginLat) * cos(dOriginLong) * (dX_ECEF - dX_0_ECEF)
	+ cos(dOriginLat) * sin(dOriginLong) * (dY_ECEF - dY_0_ECEF)
	+ sin(dOriginLat) * (dZ_ECEF - dZ_0_ECEF);

	(void)(dZ_TP);

	*pdX = dX_TP;
	*pdY = dY_TP;
}

//======================================================================

void PositionWgs84::transformFromTangentialPlane(double dX, double dY, const PositionWgs84& origin)
{
	INT32 nNoIterationSteps;
	double dX_ECEF;
	double dY_ECEF;
	double dZ_ECEF;
	double dLat, dLong;
	double dX_0_ECEF;
	double dY_0_ECEF;
	double dZ_0_ECEF;
	double dN;
	double dPhi, dLambda;
	double dP;
	INT32 nCount;


	// Konstanten setzen
	const double dGps_SemiMajorAxis	= 6378137.0;
	const double dGps_SemiMinorAxis	= 6356752.3142;
	const double dGps_Flatness		= (dGps_SemiMajorAxis - dGps_SemiMinorAxis) / dGps_SemiMajorAxis;
	const double dGps_Eccentricity2	= dGps_Flatness * (2.0 - dGps_Flatness);
	//	const double dGps_Eccentricity	= sqrt(dGps_Eccentricity2);


	// Winkel werden in Grad gegeben, zum rechnen brauchen wir rad.
	const double dOriginLat = origin.getLatitudeInRad();	// (GPS_PI / 180.0);
	const double dOriginLong = origin.getLongitudeInRad();	// (GPS_PI / 180.0);

	nNoIterationSteps = 200;
	dX_ECEF = 0.0;
	dY_ECEF = 0.0;
	dZ_ECEF = 0.0;

	// Origin of the Tangentplane in ECEF-coordinates
	dN = dGps_SemiMajorAxis /
	(sqrt(1 - (dGps_Eccentricity2 * sin(dOriginLat) * sin(dOriginLat))));

	dX_0_ECEF = (dN + 0.0) * cos(dOriginLat) * cos(dOriginLong);
	dY_0_ECEF = (dN + 0.0) * cos(dOriginLat) * sin(dOriginLong);
	dZ_0_ECEF = ((dN * (1.0 - (dGps_Eccentricity2))) + 0.0) * sin(dOriginLat);

	// see "Geographic Coordiante Transformation and
	//      Landmark Navigation" (T.Weiss)

	dLambda = dOriginLat;
	dPhi	= dOriginLong;

	dX_ECEF = -cos(dPhi) * sin(dLambda) * dX + sin(dPhi) * dY + cos(dPhi) * cos(dLambda) * 0.0 + dX_0_ECEF;
	dY_ECEF = -sin(dPhi) * sin(dLambda) * dX - cos(dPhi) * dY + sin(dPhi) * cos(dLambda) * 0.0 + dY_0_ECEF;
	dZ_ECEF =  cos(dLambda) * dX + sin(dLambda) * 0.0 + dZ_0_ECEF;


	dN = dGps_SemiMajorAxis;
	dP = sqrt(dX_ECEF * dX_ECEF + dY_ECEF * dY_ECEF);

	//////////////////////////////////////////////////////////////////////////
	// transforamtion from ECEF to geodic coordinates
	// by an iterative numeric algorithm:
	// perform following iteration until convergence
	dLambda = 0.0;
	for (nCount = 0 ; nCount < nNoIterationSteps ; nCount++)
	{
		dLambda = atan((dZ_ECEF + dGps_Eccentricity2 * dN * sin(dLambda)) / dP);
		dN = dGps_SemiMajorAxis / (sqrt(1.0 - (dGps_Eccentricity2 * sin(dLambda) * sin(dLambda))));
	}

	dLong = atan2(dY_ECEF, dX_ECEF);
	dLat = dLambda;

	setLongitudeInRad(dLong);	// *pdLong = dLong * (180.0 / GPS_PI);
	setLatitudeInRad(dLat);		// *pdLat = dLat * (180.0 / GPS_PI);
}

//======================================================================

void PositionWgs84::transformToTangentialPlaneWithHeading(const PositionWgs84& origin, double* pdX, double* pdY) const
{
	double x, y;
	transformToTangentialPlane(origin, &x, &y);
	*pdX = cos(-origin.getCourseAngleInRad()) * x - sin(-origin.getCourseAngleInRad()) * y;
	*pdY = sin(-origin.getCourseAngleInRad()) * x + cos(-origin.getCourseAngleInRad()) * y;
}

//======================================================================

void PositionWgs84::transformFromTangentialPlaneWithHeading(const double dX, const double dY, const PositionWgs84& origin)
{
	double x = cos(origin.getCourseAngleInRad()) * dX - sin(origin.getCourseAngleInRad()) * dY;
	double y = sin(origin.getCourseAngleInRad()) * dX + cos(origin.getCourseAngleInRad()) * dY;

	transformFromTangentialPlane(x, y, origin );
	setCourseAngleInRad(origin.getCourseAngleInRad());
}

//======================================================================

std::streamsize PositionWgs84::getSerializedSize_static()
{
	return std::streamsize(sizeof(uint32_t))
			+ std::streamsize(sizeof(uint64_t))
			+ std::streamsize(sizeof(uint8_t))
			+ 14 * std::streamsize(sizeof(double))
			+ std::streamsize(sizeof(uint16_t));
}

//======================================================================

std::streamsize PositionWgs84::getSerializedSize() const
{
	return getSerializedSize_static();
}

//======================================================================

bool PositionWgs84::deserialize(std::istream& is)
{
	const std::istream::pos_type startPos = is.tellg();

	ibeosdk::readBE(is, this->m_usSinceStartup);
	ibeosdk::readBE(is, this->m_timestamp);

	ibeosdk::readBE(is, this->m_deviceId);

	ibeosdk::readBE(is, this->m_latitude);
	ibeosdk::readBE(is, this->m_longitude);
	ibeosdk::readBE(is, this->m_altitudeMSL);
	ibeosdk::readBE(is, this->m_latitudeSigma);
	ibeosdk::readBE(is, this->m_longitudeSigma);
	ibeosdk::readBE(is, this->m_altitudeMSLSigma);

	ibeosdk::readBE(is, this->m_courseAngle);
	ibeosdk::readBE(is, this->m_courseAngleSigma);

	ibeosdk::readBE(is, this->m_yawAngle);
	ibeosdk::readBE(is, this->m_yawAngleSigma);
	ibeosdk::readBE(is, this->m_pitchAngle);
	ibeosdk::readBE(is, this->m_pitchAngleSigma);
	ibeosdk::readBE(is, this->m_rollAngle);
	ibeosdk::readBE(is, this->m_rollAngleSigma);

	ibeosdk::readBE(is, this->m_source);

	return !is.fail() && ((is.tellg() - startPos) == this->getSerializedSize());
}

//======================================================================

bool PositionWgs84::serialize(std::ostream& os) const
{
	const std::istream::pos_type startPos = os.tellp();

	ibeosdk::writeBE(os, this->m_usSinceStartup);
	ibeosdk::writeBE(os, this->m_timestamp);

	ibeosdk::writeBE(os, this->m_deviceId);

	ibeosdk::writeBE(os, this->m_latitude);
	ibeosdk::writeBE(os, this->m_longitude);
	ibeosdk::writeBE(os, this->m_altitudeMSL);
	ibeosdk::writeBE(os, this->m_latitudeSigma);
	ibeosdk::writeBE(os, this->m_longitudeSigma);
	ibeosdk::writeBE(os, this->m_altitudeMSLSigma);

	ibeosdk::writeBE(os, this->m_courseAngle);
	ibeosdk::writeBE(os, this->m_courseAngleSigma);

	ibeosdk::writeBE(os, this->m_yawAngle);
	ibeosdk::writeBE(os, this->m_yawAngleSigma);
	ibeosdk::writeBE(os, this->m_pitchAngle);
	ibeosdk::writeBE(os, this->m_pitchAngleSigma);
	ibeosdk::writeBE(os, this->m_rollAngle);
	ibeosdk::writeBE(os, this->m_rollAngleSigma);

	ibeosdk::writeBE(os, this->m_source);

	return !os.fail() && ((os.tellp() - startPos) == this->getSerializedSize());
}

//======================================================================

}// namespace ibeosdk

//======================================================================
