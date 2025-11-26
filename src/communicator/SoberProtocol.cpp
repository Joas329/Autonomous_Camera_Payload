#include "communicator/SoberProtocol.hpp"

#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>

#include <netinet/in.h>
#include <sys/socket.h>

#include <spdlog/spdlog.h>

#include "camera/LastFrame.hpp"
#include "communicator/CRC16.hpp"
#include "communicator/RateLimiter.hpp"

namespace sober::communicator {

bool SoberProtocol::feed(uint8_t byte, sbPacket& packet) {
  switch (fsm.state) {
    case m_State::PREAMBLE_AB:
      if (byte == SYNC_AB) {
        fsm.buffer[0] = byte;
        fsm.pos = 1;
        packet.pos = 0;
        packet.payloadLength = 0;
        fsm.state = m_State::PREAMBLE_BC;
      }
      break;
    case m_State::PREAMBLE_BC:
      if (byte == SYNC_CD) {
        fsm.buffer[fsm.pos++] = byte;
        fsm.state = m_State::CRC_LOW;
      } else {
        reset(fsm);
      }
      break;
    case m_State::CRC_LOW:
      fsm.buffer[fsm.pos++] = byte;
      fsm.state = m_State::CRC_HIGH;
      break;
    case m_State::CRC_HIGH:
      fsm.buffer[fsm.pos++] = byte;
      fsm.state = m_State::ID_LOW_SB;
      break;
    case m_State::ID_LOW_SB:
      fsm.buffer[fsm.pos++] = byte;
      fsm.state = m_State::ID_HIGH_SB;
      break;
    case m_State::ID_HIGH_SB:
      fsm.buffer[fsm.pos] = byte;
      fsm.id = u16(fsm.buffer[fsm.pos - 1], fsm.buffer[fsm.pos]);
      ++fsm.pos;
      fsm.isExtended =
          (fsm.id == static_cast<uint16_t>(CommandList::GetPictures) ||
           fsm.id == static_cast<uint16_t>(CommandList::GetIRPicture) ||
           fsm.id == static_cast<uint16_t>(CommandList::GetOptPicture));
      fsm.state = m_State::SEQ_LOW_SB;
      break;
    case m_State::SEQ_LOW_SB:
      fsm.buffer[fsm.pos++] = byte;
      fsm.state = m_State::SEQ_HIGH_SB;
      break;
    case m_State::SEQ_HIGH_SB:
      fsm.buffer[fsm.pos++] = byte;
      fsm.state = m_State::LENGTH_1;
      break;
    case m_State::LENGTH_1:
      fsm.buffer[fsm.pos++] = byte;
      fsm.state = m_State::LENGTH_2;
      break;
    case m_State::LENGTH_2:
      fsm.buffer[fsm.pos++] = byte;
      fsm.expectedLength =
          u16(fsm.buffer[fsm.pos - 2], fsm.buffer[fsm.pos - 1]);
      if (fsm.isExtended) {
        fsm.state = m_State::LENGTH_3;
      } else {
        fsm.state =
            (fsm.expectedLength == 0) ? m_State::PADDING_SB : m_State::PAYLOAD;
      }
      break;
    case m_State::LENGTH_3:
      fsm.buffer[fsm.pos++] = byte;
      fsm.state = m_State::LENGTH_4;
      break;
    case m_State::LENGTH_4:
      fsm.buffer[fsm.pos++] = byte;
      fsm.expectedLength =
          u32(fsm.buffer[fsm.pos - 4], fsm.buffer[fsm.pos - 3],
              fsm.buffer[fsm.pos - 2], fsm.buffer[fsm.pos - 1]);
      fsm.state =
          (fsm.expectedLength == 0) ? m_State::PADDING_SB : m_State::PAYLOAD;
      break;
    case m_State::PAYLOAD: {
      fsm.buffer[fsm.pos++] = byte;
      packet.payload[packet.pos++] = byte;
      const size_t expLength = fsm.isExtended ? EXT_HDR_SIZE : HDR_SIZE;
      if (fsm.pos == (expLength + fsm.expectedLength)) {
        fsm.state = m_State::PADDING_SB;
      }
      break;
    }
    case m_State::PADDING_SB:
      fsm.buffer[fsm.pos++] = byte;
      if (byte == ENDLINE) {
        std::cout << "Frame buffer (" << fsm.pos << " bytes): ";
        for (size_t i = 0; i < fsm.pos; ++i) {
          std::cout << "0x" << std::uppercase << std::hex << std::setw(2)
                    << std::setfill('0') << static_cast<int>(fsm.buffer[i])
                    << ' ';
        }
        std::cout << std::dec << "\n";

        const uint16_t rcvCRC = u16(fsm.buffer[2], fsm.buffer[3]);
        const size_t bodyLen = (fsm.pos - 1) - BODY_OFFSET;
        const uint16_t calcCrc = crc16(&fsm.buffer[BODY_OFFSET], bodyLen);

        if (calcCrc != rcvCRC) {
          reset(fsm);
          return false;
        }
        packet.id = fsm.id;
        packet.payloadLength = fsm.expectedLength;
        reset(fsm);
        return true;
      }
      break;
    default:
      break;
  }
  return false;
}

bool SoberProtocol::handleResponse(const sbPacket& packet) {
  std::span<const uint8_t> payload{packet.payload.data(), packet.payloadLength};
  switch (packet.id) {
    case static_cast<uint16_t>(CommandList::StartExperiment):
      if (onCommand_) {
        onCommand_(packet.id, payload);
      }
      return true;
    case static_cast<uint16_t>(CommandList::HeaterOn):
      if (protocolEnabled_) {
#ifdef CROSS_COMPILATION_BUILD
        return heater_.on();
#endif
      }
      break;
    case static_cast<uint16_t>(CommandList::HeaterOff):
      if (protocolEnabled_) {
#ifdef CROSS_COMPILATION_BUILD
        return heater_.off();
#endif
      }
      break;
    case static_cast<uint16_t>(CommandList::GetOptPicture):
      if (protocolEnabled_) {
        sendOptical();
        SPDLOG_INFO("RECIEVED ANOTHER COMMAND");
        return true;
      }
      break;
    case static_cast<uint16_t>(CommandList::SetBandwidth):
      setBandwidth(payload);
      break;
    case static_cast<uint16_t>(CommandList::StopExperiment):
      if (onCommand_) {
        onCommand_(packet.id, payload);
      }
      return true;
    default:
      break;
  }
  return false;
}

void SoberProtocol::sendOptical() {
  size_t atomicIdx = current_idx.load(std::memory_order_acquire) % 2;
  std::vector<uint8_t> fullImage = buffers[atomicIdx];
  const uint32_t orSize = originalSize.load(std::memory_order_acquire);

  std::size_t imgSize = fullImage.size();
  std::size_t offset = 0;

  const int maxBandwidth = bandWidth_;

  double d = static_cast<double>(imgSize) / maxBandwidth;
  auto requiredPackages = static_cast<uint16_t>(std::ceil(d));

  std::array<uint8_t, 2> bytes{};
  std::array<uint8_t, 4> bytes32{};
  uint16_t idx = 0;

  while (offset < imgSize) {
    std::vector<uint8_t> payload;
    const std::size_t chunkSize = std::min<std::size_t>(
        static_cast<size_t>(maxBandwidth), imgSize - offset);
    payload.reserve(chunkSize + 8);

    bytes = splitU16(requiredPackages);
    payload.push_back(bytes[0]);
    payload.push_back(bytes[1]);

    bytes = splitU16(idx);
    payload.push_back(bytes[0]);
    payload.push_back(bytes[1]);

    bytes32 = splitU32(orSize);
    payload.push_back(bytes32[0]);
    payload.push_back(bytes32[1]);
    payload.push_back(bytes32[2]);
    payload.push_back(bytes32[3]);

    payload.insert(payload.end(),
                   fullImage.begin() + static_cast<uint32_t>(offset),
                   fullImage.begin() + static_cast<uint32_t>(offset) +
                       static_cast<uint32_t>(chunkSize));

    opticalPackages_.push(std::move(payload));

    offset += chunkSize;
    ++idx;
  }
}

void SoberProtocol::setBandwidth(std::span<const uint8_t> payload) {
  if (payload.size() < sizeof(uint16_t)) {
    SPDLOG_WARN("[COMMUNICATOR] Bandwidth payload too small ({} bytes)",
                payload.size());
    return;
  }

  uint16_t kbps_net;
  std::memcpy(&kbps_net, payload.data(), sizeof(kbps_net));
  uint16_t kbps = ntohs(kbps_net);

  uint32_t bytes_per_sec = static_cast<uint32_t>(kbps) * 1024u;

  auto& rateLimiter = getLimiterInstance();
  rateLimiter.setRate(bytes_per_sec);

  if (::setsockopt(socketClient_, SOL_SOCKET, SO_MAX_PACING_RATE,
                   &bytes_per_sec, sizeof(bytes_per_sec)) != 0) {
    SPDLOG_WARN(
        "[COMMUNICATOR] Failed to set SO_MAX_PACING_RATE to {} KB/s: errno={} "
        "({})",
        kbps, errno, strerror(errno));
    return;
  }

  bandWidth_ = static_cast<int>(bytes_per_sec);

  SPDLOG_INFO("[COMMUNICATOR] Updated pacing rate to {} KB/s ({} bytes/sec)",
              kbps, bytes_per_sec);
}

}  // namespace sober::communicator