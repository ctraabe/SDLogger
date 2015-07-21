#ifndef UBLOX_HPP_
#define UBLOX_HPP_

#include <Arduino.h>
#include <inttypes.h>

typedef unsigned int uint;

#define kRxBufferSize (1<<7)

#define kPacketQueuSize (1<<2)
#define kPacketQueuMask (kPacketQueuSize - 1)

// The following should be at least as large as the largest expected packet.
#define kDataBufferSize (1<<6)
#define kSyncChar1 (0xb5)
#define kSyncChar2 (0x62)
#define kClassNAV (0x01)
#define kIDPosLLH (0x02)
#define kIDVelNED (0x12)
#define kIDSol (0x06)

class UBlox
{
public:

  UBlox(HardwareSerial &serial);
  void Init(void);

  bool IsAvailable(void) { return packet_queue_head_ != packet_queue_tail_; };
  uint8_t * Data(void);
  char Class(void);
  char ID(void);
  uint Length(void);
  uint8_t ChecksumA(void);
  uint8_t ChecksumB(void);

  void Pop(void) { packet_queue_tail_ = (packet_queue_tail_ + 1) & kPacketQueuMask; };
  void ProcessIncoming(void);

private:

  UBlox(void);
  void DecodeRx(void);
  void NextDataByte(uint8_t byte);

  struct UBXPacket
  {
    char ubx_class;
    char ubx_id;
    uint length;
    uint8_t checksum_a;
    uint8_t checksum_b;
  };

  HardwareSerial &serial_;

  uint8_t rx_buffer_[kRxBufferSize];
  uint rx_buffer_head_;

  uint8_t data_buffer_[kPacketQueuSize][kDataBufferSize];
  UBXPacket packet_queue_[kPacketQueuSize];
  uint packet_queue_head_;
  uint packet_queue_tail_;

  uint8_t checksum_a_;
  uint8_t checksum_b_;
};

struct UBXPosLLH
{
  uint32_t gps_ms_time_of_week;
  int32_t longitutde;
  int32_t latitude;
  int32_t height_above_ellipsoid;
  int32_t height_mean_sea_level;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
} __attribute__((packed));

struct UBXVelNED
{
  uint32_t gps_ms_time_of_week;
  int32_t velocity_north;
  int32_t velocity_east;
  int32_t velocity_down;
  uint32_t total_speed;
  uint32_t horizontal_speed;
  int32_t course;
  uint32_t speed_accuracy;
  uint32_t course_accuracy;
} __attribute__((packed));

struct UBXSol
{
  uint32_t gps_ms_time_of_week;
  int32_t fractional_time_of_week;
  int16_t gps_week;
  uint8_t gps_fix_type;
  uint8_t gps_fix_status_flags;
  int32_t ecef_x_coordinate;
  int32_t ecef_y_coordinate;
  int32_t ecef_z_coordinate;
  uint32_t coordinate_accuracy;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t velocity_accuracy;
  uint16_t position_dop;
  uint8_t reserved1;
  uint8_t number_of_satelites_used;
  uint32_t reserved2;
} __attribute__((packed));

#endif  // UBLOX_HPP_