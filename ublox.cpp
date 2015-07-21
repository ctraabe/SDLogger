#include "ublox.h"

UBlox::UBlox(HardwareSerial &serial) :
  serial_(serial),
  rx_buffer_({ 0 }),
  rx_buffer_head_(0),
  data_buffer_({ { 0 } }),
  packet_queue_head_(0),
  packet_queue_tail_(0),
  checksum_a_(0),
  checksum_b_(0)
{
}

void UBlox::Init(void)
{
  {
    // Device starts in 9600 BAUD.
    serial_.begin(9600);

    // Set the port to UART UBX @ 57600.
    uint8_t tx_buffer[28] = { 0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00,
      0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xe1, 0x00, 0x00, 0x01, 0x00,
      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xd6, 0x8d };
    serial_.write(tx_buffer, 28);

    delay(100);

    // Clear the RX buffer.
    uint8_t rx_buffer[1024];
    while (serial_.available()) serial_.read();

    serial_.end();
    delay(50);
  }

  // Reopen the port at 57600.
  serial_.begin(57600);

  {  // Request version.
    uint8_t tx_buffer[8] = { 0xb5, 0x62, 0x0a, 0x04, 0x00, 0x00, 0x0e, 0x34 };
    serial_.write(tx_buffer, 8);
  }
  {  // Configure USB for UBX input with no output.
    uint8_t tx_buffer[28] = { 0xb5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x03, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1e, 0x8c };
    serial_.write(tx_buffer, 28);
  }
  {  // Set antenna flags to 0x0b and pins to 0x380f.
    uint8_t tx_buffer[12] = { 0xb5, 0x62, 0x06, 0x13, 0x04, 0x00, 0x0b, 0x00,
      0x0f, 0x38, 0x6f, 0x4f };
    serial_.write(tx_buffer, 12);
  }
  {  // Set measurement period to 200ms (5Hz) with UTC reference.
    uint8_t tx_buffer[14] = { 0xb5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xc8, 0x00,
      0x01, 0x00, 0x00, 0x00, 0xdd, 0x68 };
    serial_.write(tx_buffer, 14);
  }
  {  // Configure TimPulse.
    uint8_t tx_buffer[28] = { 0xb5, 0x62, 0x06, 0x07, 0x14, 0x00, 0x40, 0x42,
      0x0f, 0x00, 0x90, 0x86, 0x03, 0x00, 0xff, 0x01, 0x00, 0x00, 0x32, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfd, 0x70 };
    serial_.write(tx_buffer, 28);
  }
  {  // Configure SBAS.
    uint8_t tx_buffer[16] = { 0xb5, 0x62, 0x06, 0x16, 0x08, 0x00, 0x03, 0x03,
      0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0xbd };
    serial_.write(tx_buffer, 16);
  }
  {  // Configure navigation engine.
    uint8_t tx_buffer[44] = { 0xb5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xff, 0xff,
      0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x08, 0x3c,
      0x50, 0x00, 0x32, 0x00, 0x23, 0x00, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x97, 0xfa };
    serial_.write(tx_buffer, 44);
  }
  {  // Configure navigation engine expert settings.
    uint8_t tx_buffer[48] = { 0xb5, 0x62, 0x06, 0x23, 0x28, 0x00, 0x00, 0x00,
      0x4c, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x10, 0x14, 0x00,
      0x01, 0x00, 0x00, 0x00, 0xf8, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00, 0xc9, 0xea };
    serial_.write(tx_buffer, 48);
  }
  {  // Request NAV-POSLLH message to be output every measurement cycle.
    uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02,
      0x01, 0x0e, 0x47 };
    serial_.write(tx_buffer, 11);
  }
  {  // Request NAV-SOL message to be output every measurement cycle.
    uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06,
      0x01, 0x12, 0x4f };
    serial_.write(tx_buffer, 11);
  }
  {  // Request NAV-VELNED message to be output every measurement cycle.
    uint8_t tx_buffer[11] = { 0xb5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12,
      0x01, 0x1e, 0x67 };
    serial_.write(tx_buffer, 11);
  }
}

uint8_t * UBlox::Data(void)
{
  if (IsAvailable())
    return &data_buffer_[packet_queue_tail_][0];
  else
    return NULL;
};

char UBlox::Class(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].ubx_class;
  else
    return 0;
}

char UBlox::ID(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].ubx_id;
  else
    return 0;
}

uint UBlox::Length(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].length;
  else
    return 0;
}

uint8_t UBlox::ChecksumA(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].checksum_a;
  else
    return 0;
}

uint8_t UBlox::ChecksumB(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].checksum_b;
  else
    return 0;
}

void UBlox::ProcessIncoming(void)
{
  if (!serial_.available()) return;

  static uint length = 0;
  uint8_t byte = serial_.read();

  switch (rx_buffer_head_)
  {
    case 0:  // Sync char 1
      if (byte == kSyncChar1) rx_buffer_[rx_buffer_head_++] = byte;
      break;
    case 1:  // Sync char 2
      if (byte == kSyncChar2) rx_buffer_[rx_buffer_head_++] = byte;
      else rx_buffer_head_ = 0;
      break;
    case 2:  // Class (NAV)
      checksum_a_ = 0;
      checksum_b_ = 0;
      if (byte == kClassNAV) NextDataByte(byte);
      else rx_buffer_head_ = 0;
      break;
    case 3:  // ID (PosLLH or VelNED)
      // if (byte == kIDPosLLH || byte == kIDVelNED) NextDataByte(byte);
      // else rx_buffer_head_ = 0;
      NextDataByte(byte);
      break;
    case 4:  // Length (lower byte)
      length = byte > kDataBufferSize ? 0 : byte + 8;
      NextDataByte(byte);
      break;
    default:
      if (rx_buffer_head_ < length - 2)
      {
        NextDataByte(byte);
      }
      else if (rx_buffer_head_ == length -2)
      {
        if (byte == checksum_a_) rx_buffer_head_++;
        else rx_buffer_head_ = 0;
      }
      else
      {
        if (byte == checksum_b_) DecodeRx();
        rx_buffer_head_ = 0;
      }
      break;
  }
}

void UBlox::DecodeRx(void)
{
  packet_queue_[packet_queue_head_].ubx_class = rx_buffer_[2];
  packet_queue_[packet_queue_head_].ubx_id = rx_buffer_[3];
  packet_queue_[packet_queue_head_].length = rx_buffer_[4];
  packet_queue_[packet_queue_head_].checksum_a = checksum_a_;
  packet_queue_[packet_queue_head_].checksum_b = checksum_b_;

  for (uint i = 0; i < rx_buffer_[4]; i++)
    data_buffer_[packet_queue_head_][i] = rx_buffer_[i+6];

  // Move the packet_queue head to the next slot.
  packet_queue_head_ = (packet_queue_head_ + 1) & kPacketQueuMask;
}

void UBlox::NextDataByte(uint8_t byte)
{
  rx_buffer_[rx_buffer_head_++] = byte;
  checksum_a_ += byte;
  checksum_b_ += checksum_a_;
}
