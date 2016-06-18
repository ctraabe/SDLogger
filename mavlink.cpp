#include "mavlink.h"

#include "crc16.h"

void MAVLink::ProcessIncoming(void)
{
  if (!serial_.available()) return;

  static uint length = 0;
  uint8_t byte = serial_.read();

  switch (rx_buffer_head_)
  {
    case 0:  // Sync char
      if (byte == START_OF_FRAME) rx_buffer_[rx_buffer_head_++] = byte;
      crc_.u16 = 0xFFFF;
      break;
    case 1:  // Payload length
      length = byte;
      rx_buffer_[rx_buffer_head_++] = byte;
      crc_.u16 = CRCUpdateCCITT(crc_.u16, byte);
      break;
    default:
      if (rx_buffer_head_ < length +4)
      {
        rx_buffer_[rx_buffer_head_++] = byte;
        crc_.u16 = CRCUpdateCCITT(crc_.u16, byte);
      }
      else if (rx_buffer_head_ == length + 4)
      {
        if (byte == crc_.bytes[0]) rx_buffer_head_++;
        else rx_buffer_head_ = 0;
      }
      else
      {
        if (byte == crc_.bytes[1]) DecodeRx();
        rx_buffer_head_ = 0;
      }
      break;
  }
}

void MAVLink::DecodeRx(void)
{
  packet_queue_[packet_queue_head_].length = rx_buffer_[1];
  packet_queue_[packet_queue_head_].sequence_number = rx_buffer_[2];
  packet_queue_[packet_queue_head_].system_id = rx_buffer_[3];
  packet_queue_[packet_queue_head_].message_id = rx_buffer_[4];
  packet_queue_[packet_queue_head_].system_id = rx_buffer_[5];
  packet_queue_[packet_queue_head_].crc.bytes[0] = 6 + rx_buffer_[1];
  packet_queue_[packet_queue_head_].crc.bytes[1] = 7 + rx_buffer_[1];

  for (uint i = 0; i < rx_buffer_[1]; i++)
    data_buffer_[packet_queue_head_][i] = rx_buffer_[6+i];

  // Move the packet_queue head to the next slot.
  packet_queue_head_ = (packet_queue_head_ + 1) & kPacketQueuMask;
}
