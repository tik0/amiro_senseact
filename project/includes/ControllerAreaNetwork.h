//============================================================================
// Name        : ControllerAreaNetwork.h
// Author      : tkorthals <tkorthals@cit-ec.uni-bielefeld.de>
// Description : Communication via CAN
//============================================================================


// This defines as to be the same as in ControllerAreaNetwork.h for the
// embedded processor designe

#define UPDATE_PERIOD_MSEC              MS2ST(125)

#define PERIODIC_TIMER_ID               1
#define CAN_RECEIVED_ID                 2

#define CAN_BOARD_ID_SHIFT              0x00u
#define CAN_BOARD_ID_MASK               0x07u
#define CAN_DEVICE_ID_SHIFT             0x03u
#define CAN_DEVICE_ID_MASK              0xFFu
#define CAN_INDEX_ID_SHIFT              0x03u
#define CAN_INDEX_ID_MASK               0x07u

#define CAN_LIGHT_RING_ID               1
#define CAN_POWER_MANAGEMENT_ID         2
#define CAN_DI_WHEEL_DRIVE_ID           3
#define CAN_COGNITION                   4

#define CAN_PROXIMITY_FLOOR_ID          0x51
#define CAN_ODOMETRY_ID                 0x50
#define CAN_BRIGHTNESS_ID               0x40
#define CAN_COLOR_ID(index)             (0x38 | ((index) & 0x7))
#define CAN_PROXIMITY_RING_ID(index)    (0x30 | ((index) & 0x7))
#define CAN_ACTUAL_SPEED_ID             0x20
#define CAN_SET_ODOMETRY_ID             0x11
#define CAN_TARGET_SPEED_ID             0x10
#define CAN_BROADCAST_SHUTDOWN          0x80u

#define CAN_SHUTDOWN_MAGIC              0xAA55u

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

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    /* Listen to own sending */
//    setsockopt(s, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own_msgs, sizeof(recv_own_msgs));
  }
  ~ControllerAreaNetwork() {
    close(s);
  }

 public:
  void setTargetSpeed(int v, int w) {
    /* first fill, then send the CAN frame */
    this->frame.can_id = 0;
    this->encodeDeviceId(&this->frame, CAN_TARGET_SPEED_ID);
    memcpy(&(this->frame.data[0]), &v, 4);
    memcpy(&(this->frame.data[4]), &w, 4);
    this->frame.can_dlc = 8;
    this->transmitMessage(&frame);
  }

  void getAnyFrame() {
    int nbytes = read(s, &frame, sizeof(frame));
     if (nbytes > 0) {
       printf("ID=0x%X DLC=%d data[0]=0x%X\n ",frame.can_id,frame.can_dlc,frame.data[0]);
     }
  }
  
  types::position getOdometry() {
    
    struct can_filter rfilter[1];

    /* Set the filter for the message */
    rfilter[0].can_id   = ((CAN_ODOMETRY_ID & CAN_DEVICE_ID_MASK) << CAN_DEVICE_ID_SHIFT) | CAN_DI_WHEEL_DRIVE_ID;
//    rfilter[0].can_id   = 0x383u;
    rfilter[0].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    
    /* Listen to the socket */
    int nbytes = read(s, &frame, sizeof(frame));
    /* Process the data */
    types::position robotPosition;
    
    robotPosition.x = (frame.data[0] << 8 | frame.data[1] << 16 | frame.data[2] << 24);
    robotPosition.y = (frame.data[3] << 8 | frame.data[4] << 16 | frame.data[5] << 24);
    robotPosition.f_z = (frame.data[6] << 8 | frame.data[7] << 16);
            
    
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
    
    return robotPosition;
  }

  void setOdometry(types::position robotPosition) {

    // NOTE: for robotPosition only values between [0, 2 * pi * 1e6] are allowed

    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN_SET_ODOMETRY_ID);
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

  int getSpeedRpm(int32_t &leftWheelRpm, int32_t &rightWheelRpm) {

    int returnValue = 0;
    struct can_filter rfilter[1];

    /* Set the filter for the message */
    rfilter[0].can_id   = ((CAN_ACTUAL_SPEED_ID & CAN_DEVICE_ID_MASK) << CAN_DEVICE_ID_SHIFT) | CAN_DI_WHEEL_DRIVE_ID;
    rfilter[0].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    /* Listen to the socket */
    int nbytes = read(s, &frame, sizeof(frame));
    /* Process the data */
     if (nbytes != 8) {
       memcpy(&leftWheelRpm, &(frame.data[0]), 4);
       memcpy(&rightWheelRpm, &(frame.data[4]), 4);
     } else {
       returnValue = -1;
     }

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return returnValue;

  }
  
  int getProximityFloorValue(std::vector<uint16_t> &proximityValues) {

    int returnValue = 0;
    struct can_filter rfilter[1];

    /* Set the filter for the message */
    rfilter[0].can_id   = getCanFilter(CAN_PROXIMITY_FLOOR_ID, CAN_DI_WHEEL_DRIVE_ID);
    rfilter[0].can_mask = CAN_SFF_MASK;

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    /* Listen to the socket */
    int nbytes = read(s, &frame, sizeof(frame));
    /* Process the data */
     if (nbytes != 8) {
       memcpy(&proximityValues[0], &(frame.data[0]), 2);  // Front right
       memcpy(&proximityValues[1], &(frame.data[2]), 2);  // Wheel right
       memcpy(&proximityValues[2], &(frame.data[4]), 2);  // Wheel left
       memcpy(&proximityValues[3], &(frame.data[6]), 2);  // Front left
     } else {
       returnValue = -1;
     }

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return returnValue;

  }

  int getProximityRingValue(std::vector<uint16_t> &proximityValues) {

    int returnValue = 0;
    struct can_filter rfilter[8];

    /* Set the filter for the message */
    for (int idx = 0; idx < 8; ++idx ) {
      rfilter[idx].can_id   = ((CAN_PROXIMITY_RING_ID(idx) & CAN_DEVICE_ID_MASK) << CAN_DEVICE_ID_SHIFT) | CAN_POWER_MANAGEMENT_ID;
      rfilter[idx].can_mask = CAN_SFF_MASK;
    }

    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    for (int sensorIdx = 0; sensorIdx < 8; ++sensorIdx) {
      int nbytes = read(s, &frame, sizeof(frame));
       if (nbytes != 2) {
         int index = decodeDeviceId(&frame) & 0x7;
         memcpy(&proximityValues[index], &(frame.data[0]), 2);
       } else {
         returnValue = -1;
       }
    }

    /* Disable all incomming frames */
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

    return returnValue;
  }

  void broadcastShutdown() {
      this->frame.can_id = 0;
      this->encodeDeviceId(&frame, CAN_BROADCAST_SHUTDOWN);
      const uint16_t data = CAN_SHUTDOWN_MAGIC;
      memcpy(&(this->frame.data[0]),&data,2);
      this->frame.can_dlc = 2;
      this->transmitMessage(&frame);

  }

  void setLightBrightness(uint8_t brightness) {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN_BRIGHTNESS_ID);
    memcpy(&(this->frame.data[0]),&brightness,1);
    this->frame.can_dlc = 1;
    this->transmitMessage(&frame);
  }

  void setLightColor(int index, Color color) {
    this->frame.can_id = 0;
    this->encodeDeviceId(&frame, CAN_COLOR_ID(index));
    uint8_t redColor = color.getRed();
    uint8_t greenColor = color.getGreen();
    uint8_t blueColor = color.getBlue();
    memcpy(&(this->frame.data[0]), &redColor,1);
    memcpy(&(this->frame.data[1]), &greenColor,1);
    memcpy(&(this->frame.data[2]), &blueColor,1);
    this->frame.can_dlc = 3;
    this->transmitMessage(&frame);
  }

 private:
   
  int getCanFilter(int deviceID, int boardID) {
    return ((deviceID & CAN_DEVICE_ID_MASK) << CAN_DEVICE_ID_SHIFT) | boardID;
  }
  
  void encodeDeviceId(struct can_frame *frame, int device) {
    frame->can_id |= (device & CAN_DEVICE_ID_MASK) << CAN_DEVICE_ID_SHIFT;
  }

  int decodeDeviceId(struct can_frame *frame) {
    return (frame->can_id >> CAN_DEVICE_ID_SHIFT) & CAN_DEVICE_ID_MASK;
  }

  ssize_t transmitMessage(struct can_frame *frame) {
    this->encodeBoardId(frame, CAN_COGNITION);
    return write(this->s, &this->frame, sizeof(this->frame));
  }

  void encodeBoardId(struct can_frame *frame, int board) {
    frame->can_id |= (board & CAN_BOARD_ID_MASK) << CAN_BOARD_ID_SHIFT;
  }

  struct ifreq ifr;
  struct sockaddr_can addr;
  struct can_frame frame;
  int s;
};
