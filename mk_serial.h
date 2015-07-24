#ifndef MK_SERIAL_HPP_
#define MK_SERIAL_HPP_

#include <Arduino.h>
#include <inttypes.h>

typedef unsigned int uint;

#define kRxBufferSize (1<<8)

#define kPacketQueuSize (1<<2)
#define kPacketQueuMask (kPacketQueuSize - 1)

// The following should be at least as large as the largest expected packet.
#define kDataBufferSize (1<<6)

class MKSerial
{
public:

  MKSerial(HardwareSerial &serial);
  void Init(void);

  bool IsAvailable(void) { return packet_queue_head_ != packet_queue_tail_; };
  uint8_t * Data(void);

  void Pop(void) { packet_queue_tail_ = (packet_queue_tail_ + 1) & kPacketQueuMask; };
  void ProcessIncoming(void);

private:

  MKSerial(void);
  void DecodeRx(void);

  struct MKPacket
  {
    char address;
    char label;
    uint length;
  };

  HardwareSerial &serial_;

  uint8_t rx_buffer_[kRxBufferSize];
  uint rx_buffer_head_;

  uint8_t data_buffer_[kPacketQueuSize][kDataBufferSize];
  MKPacket packet_queue_[kPacketQueuSize];
  uint packet_queue_head_;
  uint packet_queue_tail_;

  uint16_t checksum_;
};

struct FCSensorData {
  int16_t timestamp;
  int16_t accelerometer_sum[3];
  int16_t gyro_sum[3];
  uint16_t biased_pressure;
  uint8_t counter_128_hz;
  uint8_t led_on;
} __attribute__((packed));


#endif  // MK_SERIAL_HPP_