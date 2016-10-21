#ifndef UTOKYO_HPP_
#define UTOKYO_HPP_

#include <Arduino.h>

#include "union_types.h"

typedef unsigned int uint;


#define UTOKYO_QUEUE_SIZE (1<<2)
#define UTOKYO_QUEUE_MASK (UTOKYO_QUEUE_SIZE - 1)

#define UTOKYO_DATA_BUFFER_SIZE (1<<7)  // Should fit the largest payload

#define START_OF_FRAME (0xFE)
#define MESSAGE_ID_OPTICAL_FLOW (100)
#define MESSAGE_ID_OPTICAL_FLOW_RAD (106)

class UTokyo
{
public:

  UTokyo(void);

  bool UnreadData(void) { return queue_tail_ != queue_head_; };
  uint8_t * Payload(void) { return packet_queue_[queue_head_].payload; };

  uint8_t Length(void) { return packet_queue_[queue_head_].length; };
  uint8_t MessageID(void) { return packet_queue_[queue_head_].message_id; };
  uint16_t CRC(void) { return packet_queue_[queue_head_].crc; };

  bool HasOverrun(void) { return overrun_; };

  void Pop(void) { queue_head_ = (queue_head_ + 1) & UTOKYO_QUEUE_MASK; };
  void ProcessIncoming(uint8_t byte);

private:

  void DecodeRx(uint16_t crc);

  struct UTokyoPacket
  {
    uint16_t crc;
    uint16_t padding;
    uint8_t start_byte;
    uint8_t length;
    uint8_t message_id;
    uint8_t reserved;
    uint8_t payload[UTOKYO_DATA_BUFFER_SIZE];
  }  __attribute__((packed)) packet_queue_[UTOKYO_QUEUE_SIZE];

  size_t queue_head_;  // Location of oldest entry (first out)
  size_t queue_tail_;  // Location of newest entry (last out)
  size_t queue_rx_;  // Location actively modified with incoming data

  bool overrun_;  // Indicates that the queue has lost data
};

struct RicohOriginal {
  uint16_t latency;  // Latency (ms)
  uint32_t capture_time;
  uint16_t reliability;
  float velocity[3];  // (mm/frame)
  float quaternion[3];  // [q_x, q_y, q_z]
  float angular_velocity[3];  // (rad/frame)
  float position[3];  // (mm)
  uint16_t latency_ranging;  // (ms)
  float nearest_point_parameters[3];  // Distance and two angles, TBD
  float marking_point_parameters[3];  // Distance and two angles, TBD
} __attribute__((packed));

struct RicohVisualOdometry {
  uint32_t latency;  // Latency (ms)
  uint32_t capture_time;
  uint16_t reliability;
  float velocity[3];  // (mm/frame)
  float quaternion[3];  // [q_x, q_y, q_z]
  float angular_velocity[3];  // (rad/frame)
  float position[3];  // (mm)
} __attribute__((packed));

struct RicohObjectDetection {
  uint32_t latency;  // Latency (ms)
  uint32_t capture_time;
  uint16_t latency_ranging;  // (ms)
  float nearest_point_parameters[3];  // Distance and two angles, TBD
  float marking_point_parameters[3];  // Distance and two angles, TBD
} __attribute__((packed));

#endif  // UTOKYO_HPP_