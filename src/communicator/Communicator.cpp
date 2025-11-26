#include "communicator/Communicator.hpp"

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>

#include <arpa/inet.h>
#include <asm-generic/socket.h>
#include <fcntl.h>
#include <poll.h>
#include <spdlog/spdlog.h>
#include <sys/socket.h>
#include <unistd.h>

#include "communicator/RateLimiter.hpp"

using namespace std::chrono;

namespace sober::communicator {

bool Communicator::set_nonblocking(int fd) {
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags == -1) {
    return false;
  }
  if (::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
    return false;
  }
  return true;
}

void Communicator::close_fd(int& fd) {
  if (fd >= 0) {
    ::close(fd);
    fd = -1;
  }
}

int Communicator::make_listen_socket(uint16_t port) {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }

  int yes = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

#ifdef SO_REUSEPORT
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, &yes, sizeof(yes));
#endif

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  // addr.sin_addr.s_addr = htonl(LISTEN_ADDR);  // 127.0.0.1
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (::bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }
  if (::listen(fd, 1) < 0) {
    ::close(fd);
    return -1;
  }
  (void)set_nonblocking(fd);
  return fd;
}

int Communicator::accept_client(int listen_fd) {
  sockaddr_in caddr{};
  socklen_t clen = sizeof(caddr);
  int cfd = ::accept(listen_fd, reinterpret_cast<sockaddr*>(&caddr), &clen);
  if (cfd < 0) {
    return -1;
  }
  (void)set_nonblocking(cfd);
  return cfd;
}

Communicator::Communicator(uint16_t port) : port_(port) {}

Communicator::~Communicator() {
  stop();
}

void Communicator::start() {
  if (thread_.joinable()) {
    return;
  }
#ifndef MSG_NOSIGNAL
  std::signal(SIGPIPE, SIG_IGN);
#endif
  thread_ = std::jthread([this](std::stop_token st) { run(st); });
}

void Communicator::stop() {
  if (!thread_.joinable()) {
    return;
  }
  thread_.request_stop();
  thread_.join();
  close_fd(client_fd_);
  close_fd(listen_fd_);
}

void Communicator::recordSnapshot(SensorSnapshot& snapshot) {
  while (true) {
    uint64_t g1 = gSnapshot.generation.load(std::memory_order_acquire);
    if (g1 & 1) {
      continue;
    }

    snapshot.accX = gSnapshot.accX;
    snapshot.accY = gSnapshot.accY;
    snapshot.accZ = gSnapshot.accZ;
    snapshot.gyrX = gSnapshot.gyrX;
    snapshot.gyrY = gSnapshot.gyrY;
    snapshot.gyrZ = gSnapshot.gyrZ;
    snapshot.imuTemp = gSnapshot.imuTemp;
    snapshot.magX = gSnapshot.magX;
    snapshot.magY = gSnapshot.magY;
    snapshot.magZ = gSnapshot.magZ;
    snapshot.magTemp = gSnapshot.magTemp;
    snapshot.temp = gSnapshot.temp;
    snapshot.press = gSnapshot.press;
    snapshot.hum = gSnapshot.hum;
    snapshot.current3v3 = gSnapshot.current3v3;
    snapshot.voltage3v3 = gSnapshot.voltage3v3;
    snapshot.current5 = gSnapshot.current5;
    snapshot.voltage5 = gSnapshot.voltage5;
    snapshot.current12 = gSnapshot.current12;
    snapshot.voltage12 = gSnapshot.voltage12;
    snapshot.tempC1 = gSnapshot.tempC1;
    snapshot.thystC1 = gSnapshot.thystC1;
    snapshot.tosC1 = gSnapshot.tosC1;
    snapshot.tempC2 = gSnapshot.tempC2;
    snapshot.thystC2 = gSnapshot.thystC2;
    snapshot.tosC2 = gSnapshot.tosC2;
    snapshot.thermocouple_c = gSnapshot.thermocouple_c;
    snapshot.internal_c = gSnapshot.internal_c;

    uint64_t g2 = gSnapshot.generation.load(std::memory_order_acquire);
    if (g1 == g2) {
      return;
    }

    std::this_thread::yield();
  }
}

void Communicator::run(std::stop_token stoken) {
  listen_fd_ = make_listen_socket(port_);
  if (listen_fd_ < 0) {
    // SPDLOG_WARN("[COMMUNICATOR] listen failed on 127.0.0.1:{} (errno={})",
    //             port_, errno);
    return;
  }
  sockaddr_in sin{};
  socklen_t len = sizeof(sin);
  if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&sin), &len) == 0) {
    char ip[INET_ADDRSTRLEN]{};
    if (::inet_ntop(AF_INET, &sin.sin_addr, ip, sizeof(ip))) {
      SPDLOG_INFO("[COMMUNICATOR] listening on {}:{}", ip, ntohs(sin.sin_port));
    } else {
      SPDLOG_INFO("[COMMUNICATOR] listening (unknown address) on port {}",
                  ntohs(sin.sin_port));
    }
  } else {
    SPDLOG_WARN("[COMMUNICATOR] getsockname failed: {}", strerror(errno));
  }

  const uint32_t maxBandwidth = 28850;

  sbPacket packet{};
  auto nextSend = steady_clock::now() + 1s;

  constexpr int POLL_TIMEOUT_MS = 50;

  while (!stoken.stop_requested()) {
    struct pollfd fds[2];
    int nfds = 0;

    fds[nfds++] = {listen_fd_, POLLIN, 0};
    if (client_fd_ >= 0) {
      fds[nfds++] = {client_fd_, POLLIN, 0};
    }

    int rc = ::poll(fds, static_cast<uint64_t>(nfds), POLL_TIMEOUT_MS);
    if (rc < 0) {
      if (errno == EINTR) {
        continue;
      }
      SPDLOG_WARN("[COMMUNICATOR] poll error: {}", errno);
      break;
    }

    if (fds[0].revents & POLLIN) {
      int cfd = accept_client(listen_fd_);
      if (cfd >= 0) {
        if (client_fd_ >= 0) {
          SPDLOG_INFO("[COMMUNICATOR] replacing existing client");
          close_fd(client_fd_);
        }
        client_fd_ = cfd;

        if (setsockopt(client_fd_, SOL_SOCKET, SO_MAX_PACING_RATE,
                       &maxBandwidth, sizeof(maxBandwidth)) != 0) {
          SPDLOG_CRITICAL("[COMMUNICATOR] Failed to set max Limit on port");
        } else {
          soberProtocol_.setClient_(client_fd_);
        }

        sockaddr_in peer{};
        socklen_t plen = sizeof(peer);
        if (::getpeername(client_fd_, reinterpret_cast<sockaddr*>(&peer),
                          &plen) == 0) {
          char ip[INET_ADDRSTRLEN]{};
          ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
          SPDLOG_INFO("[COMMUNICATOR] Client connected from {}:{}", ip,
                      ntohs(peer.sin_port));
        } else {
          SPDLOG_INFO(
              "[COMMUNICATOR] Client connected (peer info unavailable)");
        }

        readBuffer_.fill(0);
        nextSend = steady_clock::now() + 1s;
      }
    }

    if (client_fd_ >= 0 && nfds > 1 &&
        (fds[1].revents & (POLLIN | POLLERR | POLLHUP))) {
      for (;;) {
        ssize_t n =
            ::recv(client_fd_, readBuffer_.data(), readBuffer_.size(), 0);
        if (n > 0) {
          for (ssize_t i = 0; i < n; ++i) {
            if (soberProtocol_.feed(readBuffer_[static_cast<size_t>(i)],
                                    packet)) {
              soberProtocol_.handleResponse(packet);
            }
          }
          continue;
        }
        if (n == 0) {
          SPDLOG_INFO("[COMMUNICATOR] Client disconnected");
          close_fd(client_fd_);
          outQueue_.clear();
          break;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        SPDLOG_WARN("[COMMUNICATOR] recv error: {}", errno);
        close_fd(client_fd_);
        outQueue_.clear();
        break;
      }
    }

    auto now = steady_clock::now();
    if (client_fd_ >= 0 && now >= nextSend) {
      if (telemetryEnabled_) {
        SensorSnapshot snapshot;
        recordSnapshot(snapshot);

        std::size_t pos = 0;
        std::array<int16_t, 28> vals = {
            encodeAccel(snapshot.accX),
            encodeAccel(snapshot.accY),
            encodeAccel(snapshot.accZ),
            encodeGyr(snapshot.gyrX),
            encodeGyr(snapshot.gyrY),
            encodeGyr(snapshot.gyrZ),
            encodeImuTemp(snapshot.imuTemp),
            encodeMag(snapshot.magX),
            encodeMag(snapshot.magY),
            encodeMag(snapshot.magZ),
            encodeMagTemp(snapshot.magTemp),
            encodeTemp(snapshot.temp),
            encodePress(snapshot.press),
            encodeHum(snapshot.hum),
            encodeCurrent(snapshot.current3v3),
            encodeCurrent(snapshot.current5),
            encodeCurrent(snapshot.current12),
            encodeBusVoltage(snapshot.voltage3v3),
            encodeBusVoltage(snapshot.voltage5),
            encodeBusVoltage(snapshot.voltage12),
            encodeTempC(snapshot.tempC1),
            encodeTempC(snapshot.tosC1),
            encodeTempC(snapshot.thystC1),
            encodeTempC(snapshot.tempC2),
            encodeTempC(snapshot.tosC2),
            encodeTempC(snapshot.thystC2),
            encodeTempC(snapshot.thermocouple_c),
            encodeTempC(snapshot.internal_c),
        };
        InPacket<vals.size() * 2> sendPacket{};
        sendPacket.id = static_cast<uint16_t>(CommandList::Housekeeping);

        for (int16_t v : vals) {
          const auto u = static_cast<uint16_t>(v);
          sendPacket.payload[pos++] = static_cast<uint8_t>(u >> 8);
          sendPacket.payload[pos++] = static_cast<uint8_t>(u);
        }
        sendPacket.len = static_cast<uint32_t>(pos);

        constexpr std::size_t PREAMBLE = 2;
        constexpr std::size_t CRC = 2;
        constexpr std::size_t ID = 2;
        constexpr std::size_t SEQ = 2;
        constexpr std::size_t LEN16 = 2;
        constexpr std::size_t PAD = 1;

        constexpr std::size_t PAYLOAD_BYTES = vals.size() * sizeof(int16_t);

        static constexpr std::size_t OUT_MAX =
            PREAMBLE + CRC + ID + SEQ + LEN16 + PAYLOAD_BYTES + PAD;

        std::vector<uint8_t> outbuf(OUT_MAX);
        EncodeResult res = soberProtocol_.encode(
            sendPacket, std::span<uint8_t>(outbuf.data(), outbuf.size()));

        if (res.err == EncodeResult::Ok) {
          outbuf.resize(res.bytes.size());
          outQueue_.push_back(OutMsg{.data = std::move(outbuf), .offset = 0});
        } else {
          SPDLOG_WARN("[COMMUNICATOR] encode failed (err={})",
                      static_cast<int>(res.err));
        }

        drainOpticalQueueToOut();

        // SPDLOG_INFO(
        //     "[COMMUNICATOR] Raw data: "
        //     "ACC X: {} Y: {} Z: {} "
        //     "GYR X: {} Y: {} Z: {} "
        //     "IMU Temp: {} "
        //     "MAG X: {} Y: {} Z: {} "
        //     "MAG Temp: {} "
        //     "Temp: {} Press: {} Hum: {} "
        //     "Curr 3V3: {} Curr 5V: {} Curr 12V: {} "
        //     "Volt 3V3: {} Volt 5V: {} Volt 12V: {} max1: {} max2: {}",
        //     snapshot.accX, snapshot.accY, snapshot.accZ, snapshot.gyrX,
        //     snapshot.gyrY, snapshot.gyrZ, snapshot.imuTemp, snapshot.magX,
        //     snapshot.magY, snapshot.magZ, snapshot.magTemp, snapshot.temp,
        //     snapshot.press, snapshot.hum, snapshot.current3v3,
        //     snapshot.current5, snapshot.current12, snapshot.voltage3v3,
        //     snapshot.voltage5, snapshot.voltage12, snapshot.tempC1,
        //     snapshot.tempC2);
      }

      nextSend += 1s;
    }

    if (client_fd_ >= 0 && !outQueue_.empty()) {
#ifdef MSG_NOSIGNAL
      constexpr int SEND_FLAGS = MSG_NOSIGNAL;
#else
      constexpr int SEND_FLAGS = 0;
#endif
      while (!outQueue_.empty()) {
        auto& msg = outQueue_.front();
        const std::size_t remaining = msg.data.size() - msg.offset;
        if (remaining == 0) {
          outQueue_.pop_front();
          continue;
        }

        auto& rateLimiter = getLimiterInstance();
        const std::int64_t grant = rateLimiter.available();
        if (grant <= 0) {
          break;
        }

        const std::size_t byTokens =
            static_cast<std::size_t>(std::min<std::int64_t>(
                grant, static_cast<std::int64_t>(SSIZE_MAX)));

        const std::size_t chunk = std::min<std::size_t>(remaining, byTokens);
        if (chunk == 0) {
          break;
        }

        int flags = SEND_FLAGS;
#ifdef MSG_MORE
        if (remaining > chunk) {
          flags |= MSG_MORE;
        }
#endif

        const ssize_t wrote =
            ::send(client_fd_, msg.data.data() + msg.offset, chunk, flags);

        if (wrote > 0) {
          msg.offset += static_cast<std::size_t>(wrote);
          (void)rateLimiter.tryConsume(static_cast<std::int64_t>(wrote));
          if (msg.offset == msg.data.size()) {
            outQueue_.pop_front();
          }
          continue;
        }

        if (wrote == 0 ||
            (wrote == -1 &&
             (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR))) {
          break;
        }

        SPDLOG_WARN("[COMMUNICATOR] send error: {}", errno);
        close_fd(client_fd_);
        outQueue_.clear();
        break;
      }
    }
  }

  close_fd(client_fd_);
  close_fd(listen_fd_);
}

void Communicator::drainOpticalQueueToOut() {
  static constexpr size_t kMaxOpticalPayload = 12288000;
  auto& q = soberProtocol_.getQueue();

  while (!q.empty()) {
    std::vector<uint8_t> raw = std::move(q.front());
    q.pop();

    if (raw.size() > kMaxOpticalPayload) {
      SPDLOG_WARN(
          "[COMMUNICATOR] optical package {}B exceeds {}B cap; dropping",
          raw.size(), kMaxOpticalPayload);
      continue;
    }

    const size_t dstCap = optBuf_.payload.size();
    if (raw.size() > dstCap) {
      SPDLOG_ERROR(
          "[COMMUNICATOR] payload {}B > InPacket capacity {}B; dropping",
          raw.size(), dstCap);
      continue;
    }

    optBuf_.id = static_cast<uint16_t>(CommandList::GetOptPicture);  // 0x0D
    optBuf_.len = static_cast<uint32_t>(raw.size());
    std::memcpy(optBuf_.payload.data(), raw.data(), raw.size());

    const std::size_t need = full_frame_size(optBuf_.id, optBuf_.len);
    std::vector<uint8_t> out(need);

    EncodeResult res = soberProtocol_.encode(
        optBuf_, std::span<uint8_t>(out.data(), out.size()));

    if (res.err == EncodeResult::Ok) {
      out.resize(res.bytes.size());
      outQueue_.push_back(OutMsg{.data = std::move(out), .offset = 0});
    } else {
      SPDLOG_WARN("[COMMUNICATOR] optical encode failed (err={})",
                  static_cast<int>(res.err));
    }
  }
}

}  // namespace sober::communicator
