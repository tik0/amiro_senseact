//======================================================================
/*! \file ScannerInfo.hpp
 *
 * \copydoc Copyright
 * \author Mario Brumm (mb)
 * \date Apr 26, 2012
 *///-------------------------------------------------------------------

#ifndef IBEOSDK_SCANNERINFO_HPP_SEEN
#define IBEOSDK_SCANNERINFO_HPP_SEEN

//======================================================================

#include <ibeosdk/misc/WinCompatibility.hpp>

#include <ibeosdk/datablocks/snippets/ResolutionInfo.hpp>
#include <ibeosdk/Time.hpp>
#include <ibeosdk/datablocks/snippets/Snippet.hpp>
#include <ibeosdk/inttypes.hpp>

#include <vector>

//======================================================================

namespace ibeosdk {

//======================================================================

enum ScannerType {
	ScannerType_Invalid      = 0xFFFF, // == -1
	// 0x00 reserved
	// 0x03 reserved
	ScannerType_Ecu          = 0x0004,
	// 0x05 reserved
	ScannerType_Lux          = 0x0006,
	// 0x07, 0x07 reserved
	ScannerType_Lux4         = 0x0010,
	// 0x11, 0x18, 0x19 reserved
	// 0x20, 0x21, 0x28, 0x29 reserved
	ScannerType_MiniLux      = 0x0030,
	// 0x40, 0x41, 0x42 reserved
	// 0x51 - 0x55 reserved
	// 0x60reserved
	ScannerType_ScalaB2      = 0x0062
	// 0x90 reserved
}; // ScannerType

//======================================================================

class ScannerInfo : public Snippet {
public:
	ScannerInfo();
	ScannerInfo(const ScannerInfo& src);
	virtual ~ScannerInfo();

public:
	static std::streamsize getSerializedSize_static();

public:
	virtual std::streamsize getSerializedSize() const { return getSerializedSize_static(); }
	virtual bool deserialize(std::istream& is);
	virtual bool serialize(std::ostream& os) const;

public:
	UINT8 getDeviceId() const { return m_deviceId; }
	ScannerType getScannerType() const { return m_scannerType; }
	UINT16 getScanNumber() const { return m_scanNumber; }

	float getStartAngle() const { return m_startAngle; }
	float getEndAngle() const { return m_endAngle; }

	NTPTime getScanStartTime() const { return m_scanStartTime; }
	NTPTime getScanEndTime() const { return m_scanEndTime; }

	NTPTime getScanStartTimeFromDevice() const { return m_scanStartTimeFromDevice; }
	NTPTime getScanEndTimeFromDevice() const { return m_scanEndTimeFromDevice; }

	float getFrequency() const { return m_scanFrequency; }
	float getBeamTilt() const { return m_beamTilt; }

	UINT32 getFlags() const { return m_scanFlags; }

	float getYawAngle() const { return m_yawAngle; }
	float getPitchAngle() const { return m_pitchAngle; }
	float getRollAngle() const { return m_rollAngle; }

	float getOffsetX() const { return m_offsetX; }
	float getOffsetY() const { return m_offsetY; }
	float getOffsetZ() const { return m_offsetZ; }

	const std::vector<ResolutionInfo>& getResolutionInfo() const { return m_ri; }
	std::vector<ResolutionInfo>& getResolutionInfo() { return m_ri; }

public:
	static const unsigned int nbOfResolutionInfo = 8;

protected:
	UINT8 m_deviceId;
	ScannerType m_scannerType; // UINT8
	UINT16 m_scanNumber;

	UINT32 m_scannerStatus;

	float m_startAngle; // TODO: define FLOAT32
	float m_endAngle; // TODO: define FLOAT32
	NTPTime m_scanStartTime;
	NTPTime m_scanEndTime;
	NTPTime m_scanStartTimeFromDevice;
	NTPTime m_scanEndTimeFromDevice;
	float m_scanFrequency; // TODO: define FLOAT32
	float m_beamTilt; // TODO: define FLOAT32
	UINT32 m_scanFlags;
	float m_yawAngle;
	float m_pitchAngle;
	float m_rollAngle;
	float m_offsetX;
	float m_offsetY;
	float m_offsetZ;
	std::vector<ResolutionInfo> m_ri;
}; // ScannerInfo

//======================================================================

} // namespace ibeosdk

//======================================================================

#endif // IBEOSDK_SCANNERINFO_HPP_SEEN

//======================================================================
