#ifndef MAVLINK_HPP_
#define MAVLINK_HPP_

#include <Arduino.h>
#include <inttypes.h>

#include "union_types.h"

typedef unsigned int uint;

#define kRxBufferSize (1<<8)

#define kPacketQueuSize (1<<2)
#define kPacketQueuMask (kPacketQueuSize - 1)

// The following should be at least as large as the largest expected packet.
#define kDataBufferSize (1<<6)

#define START_OF_FRAME (0xfe)
#define MESSAGE_ID_OPTICAL_FLOW (100)
#define MESSAGE_ID_OPTICAL_FLOW_RAD (106)

class MAVLink
{
public:

  MAVLink(HardwareSerial &serial);
  void Init(void);

  bool IsAvailable(void) { return packet_queue_head_ != packet_queue_tail_; };
  uint8_t * Data(void);

  void Pop(void) { packet_queue_tail_ = (packet_queue_tail_ + 1) & kPacketQueuMask; };
  void ProcessIncoming(void);

private:

  MAVLink(void);
  void DecodeRx(void);

  struct MAVLinkPacket
  {
    char system_id;
    char component_id;
    char message_id;
    uint length;
    uint sequence_number;
    union U16Bytes crc;
  };

  HardwareSerial &serial_;

  uint8_t rx_buffer_[kRxBufferSize];
  uint rx_buffer_head_;

  uint8_t data_buffer_[kPacketQueuSize][kDataBufferSize];
  MAVLinkPacket packet_queue_[kPacketQueuSize];
  uint packet_queue_head_;
  uint packet_queue_tail_;

  union U16Bytes crc_;
};

struct OpticalFlow {
  uint64_t time_usec;  //  Timestamp (UNIX)
  uint8_t sensor_id;  // Sensor ID
  int16_t flow_x;  // Flow in pixels * 10 in x-sensor direction (dezi-pixels)
  int16_t flow_y;  // Flow in pixels * 10 in y-sensor direction (dezi-pixels)
  float flow_comp_m_x;  // Flow in meters in x-sensor direction, angular-speed compensated
  float flow_comp_m_y;  // Flow in meters in y-sensor direction, angular-speed compensated
  uint8_t quality;  // Optical flow quality / confidence. 0: bad, 255: maximum quality
  float ground_distance;  // Ground distance in meters. Negative for unknown distance
} __attribute__((packed));

struct OpticalFlowRad {
  uint64_t time_usec;  // Timestamp (microseconds, synced to UNIX time or since system boot)
  uint8_t sensor_id;  // Sensor ID
  uint32_t integration_time_us;  // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
  float integrated_x;  // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
  float integrated_y;  // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
  float integrated_xgyro;  // RH rotation around X axis (rad)
  float integrated_ygyro;  // RH rotation around Y axis (rad)
  float integrated_zgyro;  // RH rotation around Z axis (rad)
  int16_t temperature;  // Temperature * 100 in centi-degrees Celsius
  uint8_t quality;  // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
  uint32_t time_delta_distance_us;  // Time in microseconds since the distance was sampled.
  float distance;  // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
} __attribute__((packed));

#endif  // MAVLINK_HPP_