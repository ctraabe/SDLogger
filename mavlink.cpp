#include "mavlink.hpp"

#include "crc16.h"

MAVLink::MAVLink(void) :
  packet_queue_({ { 0 } }),
  queue_head_(MAVLINK_QUEUE_MASK),
  queue_tail_(MAVLINK_QUEUE_MASK),
  queue_rx_(0),
  overrun_(false)
{
}

void MAVLink::ProcessIncoming(uint8_t byte)
{
  static uint8_t * rx_ptr = &packet_queue_[0].start_byte;
  static size_t bytes_processed = 0, length = 0;
  static int message_id;
  static union U16Bytes crc;

  switch (bytes_processed)
  {
    case 0:  // Sync char
      if (byte != 0xFE) return;
      break;
    case 1:  // Payload length
      if (byte > MAVLINK_DATA_BUFFER_SIZE - 6 - 2) goto RESET;
      length = byte;
      crc.u16 = 0xFFFF;
    default:
      if (bytes_processed < 6 + length)
      {
        crc.u16 = CRCUpdateCCITT(crc.u16, byte);
      }
      else if (bytes_processed == 6 + length)
      {
        uint8_t crc_extra;
        switch (packet_queue_[queue_rx_].message_id)
        {
          case 0:  // heartbeat
            crc_extra = 50;
            break;
          case 100:  // optical flow
            crc_extra = 175;
            break;
          case 106:  // optical flow rad
            crc_extra = 138;
            break;
          default:
            crc_extra = 0;
            break;
        }
        crc.u16 = CRCUpdateCCITT(crc.u16, crc_extra);
        if (byte != crc.bytes[0]) goto RESET;
      }
      else
      {
        if (byte == crc.bytes[1]) DecodeRx(crc.u16);
        goto RESET;
      }
      break;
  }
  *rx_ptr++ = byte;
  bytes_processed++;
  return;

  RESET:
  rx_ptr = &packet_queue_[queue_rx_].start_byte;
  bytes_processed = 0;
}

void MAVLink::DecodeRx(uint16_t crc)
{
  packet_queue_[queue_rx_].crc = crc;

  // Set the tail of the queue to the newly processed data.
  queue_tail_ = queue_rx_;

  // Move the packet_queue reception location to the next slot.
  queue_rx_ = (queue_rx_ + 1) & MAVLINK_QUEUE_MASK;

  if (queue_rx_ == queue_head_) overrun_ = true;
}
