#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>

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