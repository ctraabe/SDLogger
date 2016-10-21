#include "utokyo.hpp"

#include "crc16.h"

UTokyo::UTokyo(void) :
  packet_queue_({ { 0 } }),
  queue_head_(UTOKYO_QUEUE_MASK),
  queue_tail_(UTOKYO_QUEUE_MASK),
  queue_rx_(0),
  overrun_(false)
{
}

void UTokyo::ProcessIncoming(uint8_t byte)
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
      if (byte > UTOKYO_DATA_BUFFER_SIZE - 4 - 2) goto RESET;
      length = byte;
      crc.u16 = 0xFFFF;
    default:
      if (bytes_processed < 4 + length)
      {
        crc.u16 = CRCUpdateCCITT(crc.u16, byte);
      }
      else if (bytes_processed == 4 + length)
      {
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

void UTokyo::DecodeRx(uint16_t crc)
{
  packet_queue_[queue_rx_].crc = crc;

  // Set the tail of the queue to the newly processed data.
  queue_tail_ = queue_rx_;

  // Move the packet_queue reception location to the next slot.
  queue_rx_ = (queue_rx_ + 1) & UTOKYO_QUEUE_MASK;

  if (queue_rx_ == queue_head_) overrun_ = true;
}
