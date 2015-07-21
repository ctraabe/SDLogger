#include "mk_serial.h"

MKSerial::MKSerial(HardwareSerial &serial) :
  serial_(serial),
  rx_buffer_({ 0 }),
  rx_buffer_head_(0),
  data_buffer_({ { 0 } }),
  packet_queue_head_(0),
  packet_queue_tail_(0),
  checksum_(0)
{
}

void MKSerial::Init(void)
{
  serial_.begin(57600);
}

uint8_t * MKSerial::Data(void)
{
  if (IsAvailable())
    return &data_buffer_[packet_queue_tail_][0];
  else
    return NULL;
};

void MKSerial::ProcessIncoming(void)
{
  if (!serial_.available()) return;

  uint8_t byte = serial_.read();
  if (rx_buffer_head_ == 0)
  {
    // Check for start condition.
    if (byte == '#')
    {
      rx_buffer_[rx_buffer_head_++] = byte;
      checksum_ = byte;
    }
  }
  else
  {
    // Reception is ongoing.
    if (byte != '\r')
    {
      // Add byte to buffer if space remains.
      if (rx_buffer_head_ < kDataBufferSize)
      {
        rx_buffer_[rx_buffer_head_++] = byte;
        checksum_ += byte;
      }
      else
      {
        rx_buffer_head_ = 0;
      }
    }
    else
    {
      // Compute checksum to validate reception.
      checksum_ -= rx_buffer_[rx_buffer_head_ - 2];
      checksum_ -= rx_buffer_[rx_buffer_head_ - 1];
      checksum_ %= 4096;
      uint8_t checksum1 = '=' + checksum_ / 64;
      uint8_t checksum2 = '=' + checksum_ % 64;
      if ((checksum1 == rx_buffer_[rx_buffer_head_ - 2]) && (checksum2 ==
        rx_buffer_[rx_buffer_head_ - 1]))
      {
        DecodeRx();
      }
      rx_buffer_head_ = 0;
    }
  }
}

void MKSerial::DecodeRx(void)
{
  packet_queue_[packet_queue_head_].address = rx_buffer_[1];
  packet_queue_[packet_queue_head_].label = rx_buffer_[2];

  // Every 3 bytes of transmitted data is encoded into 4 bytes and is wrapped in
  // a 3-byte header and 3-byte footer, so:
  if (rx_buffer_head_ > 6)
    packet_queue_[packet_queue_head_].length = ((rx_buffer_head_ - 6) / 4) * 3;
  else
    packet_queue_[packet_queue_head_].length = 0;

  rx_buffer_head_ = 3;  // Skip the 3-byte header ('#', Addr, Cmd).
  uint i = 0;
  while (i < packet_queue_[packet_queue_head_].length)
  {
    uint8_t a, b, c, d;
    uint8_t x, y, z;

    a = rx_buffer_[rx_buffer_head_++] - '=';
    b = rx_buffer_[rx_buffer_head_++] - '=';
    c = rx_buffer_[rx_buffer_head_++] - '=';
    d = rx_buffer_[rx_buffer_head_++] - '=';

    x = (a << 2) | (b >> 4);
    y = ((b & 0x0f) << 4) | (c >> 2);
    z = ((c & 0x03) << 6) | d;

    data_buffer_[packet_queue_head_][i++] = x;
    data_buffer_[packet_queue_head_][i++] = y;
    data_buffer_[packet_queue_head_][i++] = z;
  }

  // Move the packet_queue head to the next slot.
  packet_queue_head_ = (packet_queue_head_ + 1) & kPacketQueuMask;
}
