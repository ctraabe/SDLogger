#ifndef TERA_RANGER_HPP_
#define TERA_RANGER_HPP_

#include <Arduino.h>
#include <inttypes.h>

#include "union_types.h"

typedef unsigned int uint;

#define kRxBufferSize (1<<8)

#define kPacketQueuSize (1<<2)
#define kPacketQueuMask (kPacketQueuSize - 1)

#define IR_START ('T')
#define SONAR_START ('S')

class TeraRanger
{
public:

  TeraRanger(HardwareSerial &serial);
  void Init(void);

  bool IsAvailable(void) { return packet_queue_head_ != packet_queue_tail_; };
  uint8_t * Data(void);

  uint16_t IR(void) { return packet_queue_[packet_queue_tail_].ir; };
  uint16_t Sonar(void) { return packet_queue_[packet_queue_tail_].sonar; };
  uint8_t CRC(void) { return packet_queue_[packet_queue_tail_].crc; };

  void Pop(void) { packet_queue_tail_ = (packet_queue_tail_ + 1) & kPacketQueuMask; };
  void ProcessIncoming(void);

private:

  TeraRanger(void);
  void DecodeRx(void);
  uint8_t CRC8(uint8_t * array, uint8_t len);

  struct TeraRangerPacket
  {
    uint16_t ir;
    uint16_t sonar;
    uint8_t crc;
  } __attribute__((packed));

  HardwareSerial &serial_;

  uint8_t rx_buffer_[kRxBufferSize];
  uint rx_buffer_head_;

  TeraRangerPacket packet_queue_[kPacketQueuSize];
  uint packet_queue_head_;
  uint packet_queue_tail_;

  uint8_t crc_;
};

#endif  // TERA_RANGER_HPP_