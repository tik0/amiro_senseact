//============================================================================
// Name        : ControllerAreaNetwork.h
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Communication via CAN
//============================================================================

#ifndef AMIRO_CAN_H_
#define AMIRO_CAN_H_

// The CAN::* values are defined in the Constants.h, which
// is copied from the amiro-os repository
#include <Constants.h>

#include <linux/can.h>
#include <linux/can/raw.h>  // CAN_RAW_FILTER
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <stdint.h>  // int32

#include <string.h>
#include <stdio.h>
#include <unistd.h>  // sleep

#include <Color.h>  // Color types
#include <Types.h>  // types::position

#include <vector>


using namespace std;
using namespace amiro;

class ControllerAreaNetwork {
 public:
  ControllerAreaNetwork() {
    this->filterIsPromiscuous = false;
    this->filterIsFrigid = false;

    memset(&this->ifr, 0x0, sizeof(this->ifr));
    memset(&this->addr, 0x0, sizeof(this->addr));
    memset(&this->frame, 0x0, sizeof(this->frame));

    /* open CAN_RAW socket */
    this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    /* convert interface sting "can0" into interface index */
    strcpy(this->ifr.ifr_name, "can0");
    ioctl(s, SIOCGIFINDEX, &this->ifr);

    /* setup address for bind */
    this->addr.can_ifindex = this->ifr.ifr_ifindex;
    this->addr.can_family = PF_CAN;

    /* bind socket to the can0 interface */
    ::bind(s, (struct sockaddr *)&this->addr, sizeof(addr));

    /* Enable all incomming frames */
    setPromiscuousFilter();

    /* Disable loopback */
    const int loopback = 0; /* 0 = disabled, 1 = enabled (default) */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    /* Listen to own sending */
    const int recv_own_msgs = 0; /* 0 = disabled  (default), 1 = enabled */
    setsockopt(s, SOL_CAN_RAW, ~CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
  }
  ~ControllerAreaNetwork() {
    close(s);
  }

 public:

  void setTargetSpeed(int v, int w) {
    /* first fill, then send the CAN frame */
    this->frame.can_id = 0;
    this->encodeDeviceId(&this->frame, CAN::TARGET_SPEED_ID);
    memcpy(&(this->frame.data[0]), &v, 4);
    memcpy(&(this->frame.data[4]), &w, 4);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  void getAnyFrame() {
    const int nbytes = read(s, &frame, sizeof(frame));
     if (nbytes > 0) {
       printf("ID=0x%X DLC=%d data[0]=0x%X\n ",frame.can_id,frame.can_dlc,frame.data[0]);
     }
  }
  
  types::position getOdometry(bool useFilter = false) {
    types::position robotPosition;
    uint deviceID = CAN::ODOMETRY_ID;
    uint boardID = CAN::DI_WHEEL_DRIVE_ID;
    const int32_t canIdSize = 1;
    uint canId[canIdSize] = {getCanId(deviceID, boardID)};

    setMessageFilter(canId, canIdSize, useFilter);

    /* Listen to the socket until we get a proper frame */
    for (;;) {
      const int nbytes = read(s, &frame, sizeof(frame));
      if (frame.can_id == canId[0]) {
        // TODO Make proper error handling, so that the calling function knows that the data is not correct
        if (nbytes > 0) {
          /* Process the data */
          robotPosition.x = (frame.data[0] << 8 | frame.data[1] << 16 | frame.data[2] << 24);
          robotPosition.y = (frame.data[3] << 8 | frame.data[4] << 16 | frame.data[5] << 24);
          robotPosition.f_z = (frame.data[6] << 8 | frame.data[7] << 16);

        } else {
          robotPosition.x = 0;
          robotPosition.y = 0;
          robotPosition.f_z = 0;
          break;
        }
      }
    }

    if (useFilter) {
      this->setFrigidFilter();
    }
    return robotPosition;
  }
  
  void setOdometry(types::position robotPosition) {

    // NOTE: for robotPosition only values between [0, 2 * pi * 1e6] are allowed

    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::SET_ODOMETRY_ID);
    // Cut of the first byte, which precission is not needed
    int32_t x_mm = (robotPosition.x >> 8);
    int32_t y_mm = (robotPosition.y >> 8);
    int16_t f_z_mrad = int16_t(robotPosition.f_z >> 8 );
    // Copy the data structure
    memcpy((uint8_t *)&(this->frame.data[0]), (uint8_t *)&x_mm, 3);
    memcpy((uint8_t *)&(this->frame.data[3]), (uint8_t *)&y_mm, 3);
    memcpy((uint8_t *)&(this->frame.data[6]), (uint8_t *)&f_z_mrad, 2);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  void setKinematicConstants(float Ed, float Eb) {

    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::SET_KINEMATIC_CONST_ID);
    // Copy the data structure
    memcpy((uint8_t *)&(this->frame.data[0]), (uint8_t *)&Ed, 4);
    memcpy((uint8_t *)&(this->frame.data[4]), (uint8_t *)&Eb, 4);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  void setTargetPosition(types::position &robotPosition, uint16_t targetTimeMilliSeconds) {

    this->frame.can_id = 0;
    this->encodeDeviceId(&this->frame, CAN::TARGET_POSITION_ID);
    // Cut of the first byte, which precission is not needed
    int32_t x_mm = (robotPosition.x >> 8);
    int32_t f_z_mrad = int32_t(robotPosition.f_z >> 8 );
    // Copy the data structure
    memcpy((uint8_t *)&(this->frame.data[0]), (uint8_t *)&x_mm, 3);
    memcpy((uint8_t *)&(this->frame.data[3]), (uint8_t *)&f_z_mrad, 3);
    memcpy((uint8_t *)&(this->frame.data[6]), (uint8_t *)&targetTimeMilliSeconds, 2);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  int getActualSpeed(int32_t &v, int32_t &w, bool useFilter = false) {
    int returnValue = 0;
    uint deviceID = CAN::ACTUAL_SPEED_ID;
    uint boardID = CAN::DI_WHEEL_DRIVE_ID;
    const int32_t canIdSize = 1;
    uint canId[canIdSize] = {getCanId(deviceID, boardID)};

    setMessageFilter(canId, canIdSize, useFilter);

    /* Listen to the socket until we get a proper frame */
    for (;;) {
      const int nbytes = read(s, &frame, sizeof(frame));
      if (frame.can_id == canId[0]) {
        /* Process the data */
        if (nbytes != 8) {
          memcpy(&v, &(frame.data[0]), 4);
          memcpy(&w, &(frame.data[4]), 4);
        } else {
          returnValue = -1;
          break;
        }
      }
    }

    if (useFilter) {
      this->setFrigidFilter();
    }

    return returnValue;

  }

  int getProximityFloorValue(std::vector<uint16_t> &proximityValues, bool useFilter = false) {

    int returnValue = 0;
    uint deviceID = CAN::PROXIMITY_FLOOR_ID;
    uint boardID = CAN::DI_WHEEL_DRIVE_ID;
    const int32_t canIdSize = 1;
    uint canId[canIdSize] = {getCanId(deviceID, boardID)};

    setMessageFilter(canId, canIdSize, useFilter);

    /* Listen to the socket until we get a proper frame */
    for (;;) {
      const int nbytes = read(s, &frame, sizeof(frame));
      if (frame.can_id == canId[0]) {
        /* Process the data */
        if (nbytes != 8) {
          memcpy(&proximityValues[0], &(frame.data[0]), 2);  // Front right
          memcpy(&proximityValues[1], &(frame.data[2]), 2);  // Wheel right
          memcpy(&proximityValues[2], &(frame.data[4]), 2);  // Wheel left
          memcpy(&proximityValues[3], &(frame.data[6]), 2);  // Front left
        } else {
          returnValue = -1;
        }
        break;
      }
    }

    if (useFilter) {
      this->setFrigidFilter();
    }

    return returnValue;

  }

  int getProximityRingValue(std::vector<uint16_t> &proximityValues, bool useFilter = false) {

    int returnValue = 0;
    uint deviceID = CAN::PROXIMITY_FLOOR_ID;
    uint boardID = CAN::DI_WHEEL_DRIVE_ID;
    const int32_t canIdSize = 8;
    uint canId[canIdSize];
    /* Set the filter for the message */
    for (int idx = 0; idx < canIdSize; ++idx ) {
      canId[idx] = getCanId(CAN::PROXIMITY_RING_ID(idx), CAN::POWER_MANAGEMENT_ID);
    }

    setMessageFilter(canId, canIdSize, useFilter);

    /* Listen to the socket until we get a proper frame */
    int sensorIdx = 0;
    for (;;) {
      const int nbytes = read(s, &frame, sizeof(frame));
      if (frame.can_id == canId[sensorIdx]) {
        if (nbytes != 2) {
          memcpy(&proximityValues[sensorIdx], &(frame.data[0]), 2);
        } else {
          returnValue = -1;
          break;
        }
        if (++sensorIdx >= canIdSize) {
          break;
        }
      }
    }

    if (useFilter) {
      this->setFrigidFilter();
    }

    return returnValue;
  }

  void broadcastShutdown() {
      this->frame.can_id = 0;
      this->encodeDeviceId(&frame, CAN::BROADCAST_SHUTDOWN);
      const uint16_t data = CAN::SHUTDOWN_MAGIC;
      memcpy(&(this->frame.data[0]),&data,2);
      this->frame.can_dlc = 2;
      this->transmitMessage(&frame);
  }

  void setLightBrightness(uint8_t brightness) {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::BRIGHTNESS_ID);
    memcpy(&(this->frame.data[0]),&brightness,1);
    this->frame.can_dlc = 1;
    this->transmitMessage(&frame);
  }

  void setLightColor(int index, Color color) {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::COLOR_ID(index));
    uint8_t redColor = color.getRed();
    uint8_t greenColor = color.getGreen();
    uint8_t blueColor = color.getBlue();
    memcpy(&(this->frame.data[0]), &redColor,1);
    memcpy(&(this->frame.data[1]), &greenColor,1);
    memcpy(&(this->frame.data[2]), &blueColor,1);
    this->frame.can_dlc = 3;
    this->transmitMessage(&frame);
  }

  void calibrateRingProximitySensors() {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::CALIBRATE_PROXIMITY_RING);
    this->frame.can_dlc = 0;
    this->transmitMessage(&frame);
  }

  void calibrateFloorProximitySensors() {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN::CALIBRATE_PROXIMITY_FLOOR);
    this->frame.can_dlc = 0;
    this->transmitMessage(&frame);
  }

 private:
  int setMessageFilter(uint canId[], int32_t canIdSize, bool useFilter = false) {
    if (useFilter) {
      
      if (this->filterIsPromiscuous) {
        this->setFrigidFilter();
      }

      // TODO Is it even possible to set 64 filters for the mcp2115?
      const int maxFilterSize = 64;
      struct can_filter rfilter[maxFilterSize];

      /* Set the filter for the message */
      for (int filterIdx = 0; filterIdx < canIdSize; ++filterIdx) {
        rfilter[filterIdx].can_id   = canId[filterIdx];
        rfilter[filterIdx].can_mask = CAN_SFF_MASK;
      }

      setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(can_filter) * canIdSize);
    } else if (this->filterIsFrigid) {
      this->setPromiscuousFilter();
    }
  }
  
  uint getCanId(uint deviceID, uint boardID) {
    return ((deviceID & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT) | boardID;
  }
  
  void encodeDeviceId(struct can_frame *frame, uint device) {
    frame->can_id |= (device & CAN::DEVICE_ID_MASK) << CAN::DEVICE_ID_SHIFT;
  }

  int decodeDeviceId(struct can_frame *frame) {
    return (frame->can_id >> CAN::DEVICE_ID_SHIFT) & CAN::DEVICE_ID_MASK;
  }

  ssize_t transmitMessage(struct can_frame *frame) {
    this->encodeBoardId(frame, CAN::COGNITION);
    return write(this->s, &this->frame, sizeof(this->frame));
  }

  void encodeBoardId(struct can_frame *frame, int board) {
    frame->can_id |= (board & CAN::BOARD_ID_MASK) << CAN::BOARD_ID_SHIFT;
  }

  bool filterIsFrigid;
  bool filterIsPromiscuous;

  void setFilterIsPromiscuous() {
    filterIsPromiscuous = true;
    filterIsFrigid = false;
  }

  void setFilterIsFrigid() {
    filterIsPromiscuous = false;
    filterIsFrigid = true;
  }

  inline int setFrigidFilter() {
    /* Disable all incomming frames */
    int returnValue = 0;
    if (!this->filterIsFrigid) {
      int returnValue = setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
      if(returnValue == 0) {
        setFilterIsFrigid();
      }
    }
    return returnValue;
  }

  inline int setPromiscuousFilter() {
    /* Allow all incomming frames */
    int returnValue = 0;
    if (!this->filterIsPromiscuous) {
      int returnValue = setsockopt(s, SOL_CAN_RAW, !CAN_RAW_FILTER, NULL, 0);
      if(returnValue == 0) {
        setFilterIsPromiscuous();
      }
    }
    return returnValue;
  }

  struct ifreq ifr;
  struct sockaddr_can addr;
  struct can_frame frame;
  int s;
};

#endif /* AMIRO_CAN_H_ */
