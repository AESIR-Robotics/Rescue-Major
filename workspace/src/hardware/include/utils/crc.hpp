#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <linux/can.h>

// ---------------------------------------------------------------------------
// Declarations / externs
// (all overloads are inline — no separate translation unit needed yet)
// ---------------------------------------------------------------------------

inline uint8_t calcCRC(uint8_t data, uint8_t crc);
inline uint8_t calcCRC(const uint8_t *data, size_t len, uint8_t crc = 0);
inline uint8_t calcCRC(std::initializer_list<uint8_t> inputs, uint8_t crc = 0);

// ---------------------------------------------------------------------------
// Inline definitions
// CRC-8-ATM  (polynomial 0x07)
// ---------------------------------------------------------------------------

inline uint8_t calcCRC(uint8_t data, uint8_t crc) {
  crc ^= data;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80)
      crc = (crc << 1) ^ 0x07;
    else
      crc <<= 1;
  }
  return crc;
}

inline uint8_t calcCRC(const uint8_t *data, size_t len, uint8_t crc) {
  for (size_t i = 0; i < len; i++)
    crc = calcCRC(data[i], crc);
  return crc;
}

inline uint8_t calcCRC(std::initializer_list<uint8_t> inputs, uint8_t crc) {
  for (auto data : inputs)
    crc = calcCRC(data, crc);
  return crc;
}

// Utiliza la misma macro y namespace de segmentación que tenías:
inline constexpr uint32_t can_make_id(uint8_t sender, uint8_t receiver,
                                       uint16_t channel = 0, uint8_t priority = 6) {
    return CAN_EFF_FLAG |
           ((static_cast<uint32_t>(priority & 0x07) << 26) |
            (static_cast<uint32_t>(sender) << 18) |
            (static_cast<uint32_t>(receiver) << 10) |
            (static_cast<uint32_t>(channel & 0x3FF)));
}

namespace seg {
    constexpr uint8_t HEADER_SIZE    = 1;
    constexpr uint8_t FRAME_DATA_LEN = CAN_MAX_DLEN - HEADER_SIZE; 

    constexpr uint8_t TYPE_SINGLE = 0b00;
    constexpr uint8_t TYPE_START  = 0b01;
    constexpr uint8_t TYPE_CONT   = 0b10;
    constexpr uint8_t TYPE_END    = 0b11;

    inline uint8_t make_header(uint8_t type, uint8_t seq) {
        return static_cast<uint8_t>((type & 0x03) << 6) | (seq & 0x3F);
    }
    inline uint8_t frame_type(uint8_t header) { return (header >> 6) & 0x03; }
    inline uint8_t frame_seq (uint8_t header) { return  header       & 0x3F; }
} 