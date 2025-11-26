#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <queue>
#include <span>

#include <spdlog/spdlog.h>

#include "communicator/CRC16.hpp"
#include "communicator/Heater.hpp"

namespace sober::communicator {

inline constexpr uint8_t SYNC_AB = 0xAB;
inline constexpr uint8_t SYNC_CD = 0xCD;
inline constexpr uint8_t ENDLINE = 0x0A;

inline constexpr size_t BUF_SIZE = 8192;
inline constexpr size_t HDR_SIZE = 10;
inline constexpr size_t EXT_HDR_SIZE = 12;

inline constexpr size_t BODY_OFFSET = 4;

inline uint16_t u16(uint8_t high, uint8_t low) {
  return static_cast<uint16_t>((static_cast<uint16_t>(high) << 8) |
                               static_cast<uint16_t>(low));
}

inline uint32_t u32(uint8_t byte0, uint8_t byte1, uint8_t byte2,
                    uint8_t byte3) {
  return (static_cast<uint32_t>(byte0) << 24) |
         (static_cast<uint32_t>(byte1) << 16) |
         (static_cast<uint32_t>(byte2) << 8) | static_cast<uint32_t>(byte3);
}

static inline void put16be(uint8_t* p, uint16_t v) noexcept {
  p[0] = static_cast<uint8_t>(v >> 8);
  p[1] = static_cast<uint8_t>(v);
}
static inline void put32be(uint8_t* p, uint32_t v) noexcept {
  p[0] = static_cast<uint8_t>(v >> 24);
  p[1] = static_cast<uint8_t>(v >> 16);
  p[2] = static_cast<uint8_t>(v >> 8);
  p[3] = static_cast<uint8_t>(v);
}

constexpr std::array<uint8_t, 2> splitU16(uint16_t value) {
  constexpr uint8_t SHIFT_BITS = 8U;
  constexpr uint8_t SHIFT_MASK = 0xFFU;
  return std::array<uint8_t, 2>{
      static_cast<uint8_t>(value >> SHIFT_BITS),  // high
      static_cast<uint8_t>(value & SHIFT_MASK)    // low
  };
}

constexpr std::array<uint8_t, 4> splitU32(uint32_t value) {
  return {
      static_cast<uint8_t>(value >> 24),           // byte 3: highest
      static_cast<uint8_t>((value >> 16) & 0xFF),  // byte 2
      static_cast<uint8_t>((value >> 8) & 0xFF),   // byte 1
      static_cast<uint8_t>(value & 0xFF)           // byte 0: lowest
  };
}

static inline std::size_t body_header_size(uint16_t id) noexcept {
  bool isExtended =
      (id == static_cast<uint16_t>(0x0B) || id == static_cast<uint16_t>(0x0C) ||
       id == static_cast<uint16_t>(0x0D));
  return 2 + 2 + (isExtended ? 4 : 2);  // id + seq + len
}
static inline std::size_t full_frame_size(uint16_t id,
                                          std::size_t payload_len) noexcept {
  return 2 + 2 + body_header_size(id) + payload_len +
         1;  // preamble+crc+body+padding
}

struct sbPacket {
  uint16_t id{0};
  std::array<uint8_t, BUF_SIZE> payload{};
  size_t payloadLength{};
  size_t pos{0};
};

template <std::size_t N>
struct InPacket {
  std::array<uint8_t, N> payload{};
  uint32_t len = 0;
  uint16_t id = 0;
};

enum class CommandList : uint16_t {
  StartExperiment = 0x01,
  Housekeeping = 0x02,
  HeaterOn = 0x03,
  HeaterOff = 0x04,
  BothCamerasOn = 0x05,
  BothCamerasOff = 0x06,
  IrCameraOn = 0x07,
  IrCameraOff = 0x08,
  OptCameraOn = 0x09,
  OptCameraOff = 0x0A,
  GetPictures = 0x0B,
  GetIRPicture = 0x0C,
  GetOptPicture = 0x0D,
  SetPackageSize = 0x0E,
  SelfTest = 0x0F,
  Reset = 0x10,
  SyncUtc = 0x11,
  SetIRExposure = 0x12,
  SetOptExposure = 0x13,
  SetIRPicSize = 0x14,
  SetIRPixelDepth = 0x15,
  SetOptPixelDepth = 0x16,
  StopExperiment = 0x17,
  SetBandwidth = 0x18,
};

struct EncodeResult {
  std::span<const uint8_t> bytes;
  enum Err {
    Ok = 0,
    OutputTooSmall,
    PayloadLenExceedsArray,
    PayloadTooLargeFor16Bit
  } err = Ok;
};

class SoberProtocol {
 public:
  SoberProtocol() = default;

  bool feed(uint8_t byte, sbPacket& packet);

  bool handleResponse(const sbPacket& packet);

  void setClient_(int& fd) { socketClient_ = fd; }

  template <std::size_t N>
  EncodeResult encode(const InPacket<N>& input, std::span<uint8_t> out) {
    bool isExtended =
        (input.id == static_cast<uint16_t>(CommandList::GetPictures) ||
         input.id == static_cast<uint16_t>(CommandList::GetIRPicture) ||
         input.id == static_cast<uint16_t>(CommandList::GetOptPicture));
    if (input.len > N) {
      return {.bytes = {}, .err = EncodeResult::PayloadLenExceedsArray};
    }
    if (!isExtended && input.len > 0xFFFFU) {
      return {.bytes = {}, .err = EncodeResult::PayloadTooLargeFor16Bit};
    }

    const std::size_t need = full_frame_size(input.id, input.len);
    if (out.size() < need) {
      return {.bytes = {}, .err = EncodeResult::OutputTooSmall};
    }

    put16be(out.subspan(0).data(), 0xABCD);
    put16be(out.data() + 2, 0x0000);
    put16be(out.subspan(4).data(), input.id);
    seq++;
    put16be(out.subspan(6).data(), seq);

    size_t headerSize{0};
    if (isExtended) {
      headerSize = 12;
      put32be(out.subspan(8).data(), input.len);
    } else {
      headerSize = 10;
      put16be(out.subspan(8).data(), static_cast<uint16_t>(input.len));
    }

    std::memcpy(out.data() + headerSize, input.payload.data(), input.len);

    const std::size_t bodySize =
        headerSize - BODY_OFFSET + static_cast<std::size_t>(input.len);
    const uint16_t crc = crc16(out.data() + 4, bodySize);
    put16be(out.data() + 2, crc);

    out[headerSize + input.len] = 0x0A;

    return {.bytes = std::span<const uint8_t>(out.data(), need),
            .err = EncodeResult::Ok};
  }

  void setCommandHandler(
      std::function<void(uint16_t, std::span<const uint8_t>)> handler) {
    onCommand_ = std::move(handler);
  }

  void enableProtocol(bool on) {
    protocolEnabled_.store(on, std::memory_order_release);
  }

  void sendOptical();

  void setBandwidth(std::span<const uint8_t> payload);

  std::queue<std::vector<uint8_t>>& getQueue() { return opticalPackages_; }

 private:
  enum class m_State : uint8_t {
    PREAMBLE_AB,
    PREAMBLE_BC,
    CRC_LOW,
    CRC_HIGH,
    ID_LOW_SB,
    ID_HIGH_SB,
    SEQ_LOW_SB,
    SEQ_HIGH_SB,
    LENGTH_1,
    LENGTH_2,
    LENGTH_3,
    LENGTH_4,
    PAYLOAD,
    PADDING_SB
  };

  struct m_SoberState {
    m_State state = m_State::PREAMBLE_AB;
    std::array<uint8_t, BUF_SIZE> buffer{};
    size_t pos{0};
    uint32_t expectedLength{};
    uint16_t id{};
    bool isExtended{false};
  };

  static void reset(m_SoberState& fsm) {
    fsm.state = m_State::PREAMBLE_AB;
    fsm.pos = 0;
    fsm.expectedLength = 0;
    fsm.id = 0;
    fsm.isExtended = false;
  }

#ifdef CROSS_COMPILATION_BUILD
  Heater heater_;
#endif

  m_SoberState fsm;
  uint16_t seq{0};

  int socketClient_{-1};
  int bandWidth_{28750};

  std::function<void(uint16_t, std::span<const uint8_t>)> onCommand_;
  std::atomic<bool> protocolEnabled_{false};

  std::queue<std::vector<uint8_t>> opticalPackages_;
};

}  // namespace sober::communicator