#pragma once

#include <cstdint>
#include <vector>

namespace sober::communicator {

constexpr uint16_t CRC16_CCITT_POLY = 0x1021;
constexpr uint16_t CRC16_CCITT_INIT = 0xFFFF;

inline uint16_t crc16(const uint8_t* data, size_t len) noexcept {
  uint16_t crc = CRC16_CCITT_INIT;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (int j = 0; j < 8; ++j) {
      uint32_t temp = ((crc & 0x8000U) ? (static_cast<uint32_t>(crc) << 1U) ^
                                             CRC16_CCITT_POLY
                                       : (static_cast<uint32_t>(crc) << 1U));
      crc = static_cast<uint16_t>(temp);
    }
  }
  return crc;
}

inline uint16_t crc16(const std::vector<uint8_t>& data) noexcept {
  return crc16(data.data(), data.size());
}

}  // namespace sober::communicator
