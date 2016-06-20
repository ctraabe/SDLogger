#include "mavlink.h"

#include "crc16.h"

MAVLink::MAVLink(HardwareSerial &serial) :
  serial_(serial),
  rx_buffer_({ 0 }),
  rx_buffer_head_(0),
  data_buffer_({ { 0 } }),
  packet_queue_({ { 0 } }),
  packet_queue_head_(0),
  packet_queue_tail_(0),
  crc_({ 0 })
{
}

void MAVLink::Init(void)
{
  serial_.begin(115200);
}

uint8_t * MAVLink::Data(void)
{
  if (IsAvailable())
    return &data_buffer_[packet_queue_tail_][0];
  else
    return NULL;
};

char MAVLink::SystemID(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].system_id;
  else
    return 0;
}

char MAVLink::ComponentID(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].component_id;
  else
    return 0;
}

char MAVLink::MessageID(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].message_id;
  else
    return 0;
}

uint MAVLink::Length(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].length;
  else
    return 0;
}

uint MAVLink::SequenceNumber(void)
{
  if (IsAvailable())
    return packet_queue_[packet_queue_tail_].sequence_number;
  else
    return 0;
}

union U16Bytes MAVLink::CRC(void)
{
  return packet_queue_[packet_queue_tail_].crc;
}


void MAVLink::ProcessIncoming(void)
{
  if (!serial_.available()) return;

  static uint length = 0;
  uint8_t byte = serial_.read();

  switch (rx_buffer_head_)
  {
    case 0:  // Sync char
      if (byte == 0xFE) rx_buffer_[rx_buffer_head_++] = byte;
      crc_.u16 = 0xFFFF;
      break;
    case 1:  // Payload length
      length = byte;
      rx_buffer_[rx_buffer_head_++] = byte;
      crc_.u16 = CRCUpdateCCITT(crc_.u16, byte);
      break;
    default:
      if (rx_buffer_head_ < length + 6)
      {
        rx_buffer_[rx_buffer_head_++] = byte;
        crc_.u16 = CRCUpdateCCITT(crc_.u16, byte);
      }
      else if (rx_buffer_head_ == length + 6)
      {
        uint8_t crc_extra;
        switch (rx_buffer_[5])
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
        crc_.u16 = CRCUpdateCCITT(crc_.u16, crc_extra);
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
  packet_queue_[packet_queue_head_].component_id = rx_buffer_[4];
  packet_queue_[packet_queue_head_].message_id = rx_buffer_[5];
  packet_queue_[packet_queue_head_].crc.bytes[0] = crc_.bytes[0];
  packet_queue_[packet_queue_head_].crc.bytes[1] = crc_.bytes[1];

  for (uint i = 0; i < rx_buffer_[1]; i++)
    data_buffer_[packet_queue_head_][i] = rx_buffer_[6+i];

  // Move the packet_queue head to the next slot.
  packet_queue_head_ = (packet_queue_head_ + 1) & kPacketQueuMask;
}
