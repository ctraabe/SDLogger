#ifndef MAVLINK_HPP_
#define MAVLINK_HPP_

#include <Arduino.h>

#include "union_types.h"

typedef unsigned int uint;


#define MAVLINK_QUEUE_SIZE (1<<2)
#define MAVLINK_QUEUE_MASK (MAVLINK_QUEUE_SIZE - 1)

#define MAVLINK_DATA_BUFFER_SIZE (1<<7)  // Should fit the largest payload

#define START_OF_FRAME (0xFE)
#define MESSAGE_ID_OPTICAL_FLOW (100)
#define MESSAGE_ID_OPTICAL_FLOW_RAD (106)

class MAVLink
{
public:

  MAVLink(void);

  bool UnreadData(void) { return queue_tail_ != queue_head_; };
  uint8_t * Payload(void) { return packet_queue_[queue_head_].payload; };

  uint8_t ComponentID(void) { return packet_queue_[queue_head_].component_id; };
  uint8_t Length(void) { return packet_queue_[queue_head_].length; };
  uint8_t MessageID(void) { return packet_queue_[queue_head_].message_id; };
  uint8_t SequenceNumber(void) { return packet_queue_[queue_head_].sequence_number; };
  uint8_t SystemID(void) { return packet_queue_[queue_head_].system_id; };
  uint16_t CRC(void) { return packet_queue_[queue_head_].crc; };

  bool HasOverrun(void) { return overrun_; };

  void Pop(void) { queue_head_ = (queue_head_ + 1) & MAVLINK_QUEUE_MASK; };
  void ProcessIncoming(uint8_t byte);

private:

  void DecodeRx(void);

  struct MAVLinkPacket
  {
    uint16_t crc;
    uint8_t start_byte;
    uint8_t length;
    uint8_t sequence_number;
    uint8_t system_id;
    uint8_t component_id;
    uint8_t message_id;
    uint8_t payload[MAVLINK_DATA_BUFFER_SIZE];
  }  __attribute__((packed)) packet_queue_[MAVLINK_QUEUE_SIZE];

  size_t queue_head_;  // Location of oldest entry (first out)
  size_t queue_tail_;  // Location of newest entry (last out)
  size_t queue_rx_;  // Location actively modified with incoming data

  bool overrun_;  // Indicates that the queue has lost data
};

struct OpticalFlow {
  uint64_t time_usec;  // Timestamp (UNIX)
  float flow_comp_m_x;  // Flow in meters in x-sensor direction, angular-speed compensated
  float flow_comp_m_y;  // Flow in meters in y-sensor direction, angular-speed compensated
  float ground_distance;  // Ground distance in meters. Positive value: distance known. Negative value: Unknown distance
  int16_t flow_x;  // Flow in pixels * 10 in x-sensor direction (dezi-pixels)
  int16_t flow_y;  // Flow in pixels * 10 in y-sensor direction (dezi-pixels)
  uint8_t sensor_id;  // Sensor ID
  uint8_t quality;  // Optical flow quality / confidence. 0: bad, 255: maximum quality
} __attribute__((packed));

struct OpticalFlowRad {
  uint64_t time_usec;  // Timestamp (microseconds, synced to UNIX time or since system boot)
  uint32_t integration_time_us;  // Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the.
  float integrated_x;  // Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)
  float integrated_y;  // Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)
  float integrated_xgyro;  // RH rotation around X axis (rad)
  float integrated_ygyro;  // RH rotation around Y axis (rad)
  float integrated_zgyro;  // RH rotation around Z axis (rad)
  uint32_t time_delta_distance_us;  // Time in microseconds since the distance was sampled.
  float distance;  // Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance.
  int16_t temperature;  // Temperature * 100 in centi-degrees Celsius
  uint8_t sensor_id;  // Sensor ID
  uint8_t quality;  // Optical flow quality / confidence. 0: no valid flow, 255: maximum quality
} __attribute__((packed));

#endif  // MAVLINK_HPP_