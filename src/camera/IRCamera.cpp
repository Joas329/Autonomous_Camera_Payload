#include "camera/IRCamera.hpp"

#include <PvBase.h>
#include <PvDevice.h>
#include <PvDeviceInfoGEV.h>
#include <PvDeviceInfoU3V.h>
#include <PvGenEnum.h>
#include <PvGenEnum.h>  // PvGenEnum, PvGenEnumEntry
#include <PvGenInteger.h>
#include <PvGenParameterArray.h>
#include <PvGenString.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvSystem.h>
#include <PvTypes.h>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>

#include <spdlog/spdlog.h>

#include "logger/ImageLogger.hpp"

namespace {
std::string NowTimestampUS() {
  using namespace std::chrono;
  const auto now = system_clock::now();
  const auto t = system_clock::to_time_t(now);
  const auto us = duration_cast<microseconds>(now.time_since_epoch()) % 1000000;

  std::tm tm{};
#ifdef _WIN32
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << "_" << std::setw(6)
      << std::setfill('0') << us.count();
  return oss.str();  // e.g. 20251004_121314_123456
}
}  // namespace

namespace sober::camera {

bool IRCamera::start() {
  // irCameraThread_ =
  //     std::jthread([this](std::stop_token stoken) { this->run(stoken); });
  irCameraThread_ =
      std::jthread([this](std::stop_token stoken) { this->run_test(stoken); });
  return true;
}

void IRCamera::stop() {
  irCameraThread_.request_stop();

  cleanupEbus();

  cv_.notify_all();
}

void IRCamera::run_test(std::stop_token stoken) {
  PvResult result;
  m_system = new PvSystem();
  m_system->Find();

  constexpr std::string_view TELOPS_ID = {"iPORT-CL-U3-PT03-CL0UP02-128xU"};

  if (m_system->GetInterfaceCount() == 0) {
    SPDLOG_ERROR("[IR CAMERA] No ir camera found");
    return;
  }

  for (uint32_t i = 0; i < m_system->GetInterfaceCount(); ++i) {
    const PvInterface* iface = m_system->GetInterface(i);

    for (uint32_t j = 0; j < iface->GetDeviceCount(); ++j) {
      m_pvDeviceInfo = iface->GetDeviceInfo(j);
      std::string name = m_pvDeviceInfo->GetDisplayID().GetAscii();
      if (name.find(TELOPS_ID) != std::string::npos) {
        SPDLOG_INFO("[IR CAMERA] Found: {}", name);
      }
    }
  }

  ///////////////////////

  // connect to the camera
  m_device = PvDevice::CreateAndConnect(m_pvDeviceInfo, &result);
  if (!m_device || !result.IsOK()) {
    SPDLOG_ERROR("[IR Camera] Could not connect");
  }
  ///////////////////////

  // configure params
  PvGenParameterArray* params = m_device->GetParameters();
  if (!params) {
    SPDLOG_ERROR("[Could not fetch parameters]");
  }

  // Set pixel format to Mono16
  PvGenEnum* pixelFormat = dynamic_cast<PvGenEnum*>(params->Get("PixelFormat"));
  if (pixelFormat) {
    result = pixelFormat->SetValue("Mono16");
  }

  // Set acquisition mode to single frame
  PvGenEnum* acquisitionMode =
      dynamic_cast<PvGenEnum*>(params->Get("AcquisitionMode"));
  if (acquisitionMode) {
    result = acquisitionMode->SetValue("SingleFrame");
  }

  //Turn off test pattern
  PvGenEnum* testPattern = dynamic_cast<PvGenEnum*>(params->Get("TestPattern"));
  if (testPattern) {
    result = testPattern->SetValue("Off");
  }

  PvGenInteger* width = dynamic_cast<PvGenInteger*>(params->Get("Width"));
  PvGenInteger* height = dynamic_cast<PvGenInteger*>(params->Get("Height"));
  PvGenInteger* offsetX = dynamic_cast<PvGenInteger*>(params->Get("OffsetX"));
  PvGenInteger* offsetY = dynamic_cast<PvGenInteger*>(params->Get("OffsetY"));

  if (width) {
    width->SetValue(m_imageWidth);
  }
  if (height) {
    height->SetValue(m_imageHeight);
  }
  if (offsetX) {
    offsetX->SetValue(0);
  }
  if (offsetY) {
    offsetY->SetValue(0);
  }
  //////////////////////

  // GenICam commands
  auto* acqStart = dynamic_cast<PvGenCommand*>(params->Get("AcquisitionStart"));
  auto* acqStop = dynamic_cast<PvGenCommand*>(params->Get("AcquisitionStop"));
  if (!acqStart) {
    SPDLOG_ERROR("[IR CAMERA] AcquisitionStart not available");
    return;
  }
  //////////////////////

  // open stream
  PvResult streamResult;
  m_stream =
      PvStream::CreateAndOpen(m_pvDeviceInfo->GetConnectionID(), &streamResult);
  if (!m_stream || !streamResult.IsOK()) {
    SPDLOG_ERROR("[IR CAMERA] Could not open stream: {}",
                 streamResult.GetCodeString().GetAscii());
    return;
  }
  //////////////////////

  // Create stream buffers
  const uint32_t payloadSize = m_device->GetPayloadSize();
  const uint32_t maxStreamBuffers = m_stream->GetQueuedBufferMaximum();
  const uint32_t count = std::min(maxStreamBuffers, m_maxStreamBuffers);
  m_bufferList.reserve(count);

  for (uint32_t i = 0; i < count; ++i) {
    PvBuffer* buf = new PvBuffer();
    buf->Alloc(payloadSize);
    m_bufferList.push_back(buf);
  }

  for (PvBuffer* buf : m_bufferList) {
    m_stream->QueueBuffer(buf);
  }
  /////////////////////

  // init chunk header

  ////////////////////

  // enable stream
  PvResult enableResult = m_device->StreamEnable();
  if (!enableResult.IsOK()) {
    SPDLOG_ERROR("[IR CAMERA] Unable to enable stream: {}",
                 enableResult.GetCodeString().GetAscii());
    return;
  }

  std::filesystem::create_directories(m_outputDir);
  uint64_t frame_index = 0;
  //////////////////

  std::vector<uint8_t> frameBytes;

  uint64_t seen{};
  while (!stoken.stop_requested()) {
    ticker_.waitNext(seen);
    SPDLOG_INFO("[IR Camera] Tick happened");

    if (acqStart) {
      PvResult r = acqStart->Execute();
      if (!r.IsOK()) {
        SPDLOG_WARN("[IR CAMERA] AcquisitionStart failed: {}",
                    r.GetCodeString().GetAscii());
        continue;
      }
    }

    PvBuffer* buffer = nullptr;
    PvResult opRes;
    PvResult rv = m_stream->RetrieveBuffer(&buffer, &opRes, 1000);

    if (rv.IsOK() && opRes.IsOK() && buffer &&
        buffer->GetPayloadType() == PvPayloadTypeImage) {
      PvImage* img = buffer->GetImage();

      const uint8_t* src = static_cast<const uint8_t*>(img->GetDataPointer());
      const size_t sz =
          img->GetImageSize();  // equals width*height*2 for Mono16

      frameBytes.assign(src, src + sz);

      SPDLOG_INFO("[IR CAMERA] Captured {} bytes ({}x{}, {} bpp)", sz,
                  img->GetWidth(), img->GetHeight(), img->GetBitsPerPixel());

      const std::string sync_ts =
          NowTimestampUS();  // or your external synchronized timestamp
      const std::string base =
          fmt::format("frame_{:06d}", frame_index++);  // or your own naming
      const std::string path = (std::filesystem::path(m_outputDir) /
                                fmt::format("{}_{}.hcc", base, sync_ts))
                                   .string();

      // Save using your Telops writer. Pass nullptr to use your internal header update path.
      if (!SaveBufferAsTelopsRaw(buffer, path)) {
        SPDLOG_WARN("[Telops] SaveBufferAsTelopsRaw failed: {}", path);
      } else {
        SPDLOG_DEBUG("[Telops] Saved {}", path);
      }

      SPDLOG_INFO("[IR CAMERA] Captured {} bytes ({}x{}, {} bpp)", sz,
                  img->GetWidth(), img->GetHeight(), img->GetBitsPerPixel());

    } else {
      if (!rv.IsOK())
        SPDLOG_WARN("[IR CAMERA] RetrieveBuffer failed: {}",
                    rv.GetCodeString().GetAscii());
      if (!opRes.IsOK())
        SPDLOG_WARN("[IR CAMERA] Buffer op result not OK: {}",
                    opRes.GetCodeString().GetAscii());
    }

    if (buffer)
      m_stream->QueueBuffer(buffer);
  }
  if (acqStop)
    acqStop->Execute();

  // Abort + flush queued buffers
  m_stream->AbortQueuedBuffers();
  {
    PvBuffer* b = nullptr;
    PvResult op;
    while (m_stream->RetrieveBuffer(&b, &op).IsOK() && b) { /* discard */
    }
  }

  m_device->StreamDisable();

  for (auto* b : m_bufferList) {
    delete b;
  }
  m_bufferList.clear();

  PvStream::Free(m_stream);
  m_stream = nullptr;
  m_device->Disconnect();
  PvDevice::Free(m_device);
  m_device = nullptr;
  delete m_system;
  m_system = nullptr;
}

void IRCamera::run(std::stop_token stoken) {
  imglog::ImageLogger<64> logger(imglog::FlushPolicy::Never);
  logger.start();

  PvSystem sys;
  sys.SetDetectionTimeout(2000);
  PvResult r = sys.Find();
  if (!r.IsOK()) {
    SPDLOG_ERROR("[IR CAMERA] eBUS Find() failed: {}",
                 r.GetCodeString().GetAscii());
    return;
  }

  auto ci_lower = [](const PvString& s) {
    std::string out = s.GetAscii();
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return out;
  };

  const PvDeviceInfo* picked = nullptr;
  const PvDeviceInfo* first = nullptr;

  for (uint32_t i = 0; i < sys.GetInterfaceCount(); ++i) {
    const PvInterface* iface = sys.GetInterface(i);
    for (uint32_t d = 0; d < iface->GetDeviceCount(); ++d) {
      const PvDeviceInfo* di = iface->GetDeviceInfo(d);
      if (!di)
        continue;

      // Log what we see
      const char* proto =
          (dynamic_cast<const PvDeviceInfoGEV*>(di) != nullptr) ? "GigE Vision"
          : (dynamic_cast<const PvDeviceInfoU3V*>(di) != nullptr)
              ? "USB3 Vision"
              : "Unknown";
      SPDLOG_INFO("[IR CAMERA] Found: {} [{}]  conn='{}'",
                  di->GetDisplayID().GetAscii(), proto,
                  di->GetConnectionID().GetAscii());

      if (!first)
        first = di;

      // Match Pleora USB3 bridge by VID:PID in ConnectionID (Linux-friendly)
      // Examples often look like "...28B7:0001..." or similar; compare lowercase
      std::string cid = ci_lower(di->GetConnectionID());
      if (cid.find("28b7") != std::string::npos &&
          cid.find("0001") != std::string::npos) {
        picked = di;
        break;
      }
    }
    if (picked)
      break;
  }

  if (!picked) {
    SPDLOG_WARN(
        "[IR CAMERA] Pleora 28b7:0001 not found; falling back to first "
        "enumerated device.");
    picked = first;
  }
  if (!picked) {
    SPDLOG_ERROR("[IR CAMERA] No GenICam device found.");
    return;
  }

  PvString connID = picked->GetConnectionID();
  SPDLOG_INFO("[IR CAMERA] Selected device: {}",
              picked->GetDisplayID().GetAscii());

  // 2) Connect
  PvResult cres;
  std::unique_ptr<PvDevice, void (*)(PvDevice*)> dev(
      PvDevice::CreateAndConnect(connID, &cres), [](PvDevice* p) {
        if (p)
          PvDevice::Free(p);
      });

  if (!cres.IsOK() || !dev) {
    SPDLOG_ERROR("[IR CAMERA] CreateAndConnect failed: {}",
                 cres.GetCodeString().GetAscii());
    return;
  }

  // 3) Print sanity checks
  PvGenParameterArray* par = dev->GetParameters();

  auto getStr = [&](const char* name) -> std::string {
    if (auto* s = dynamic_cast<PvGenString*>(par->Get(name))) {
      PvString v;
      s->GetValue(v);
      return v.GetAscii();
    }
    return {};
  };
  auto getI64 = [&](const char* name) -> int64_t {
    if (auto* i = dynamic_cast<PvGenInteger*>(par->Get(name))) {
      int64_t v = 0;  // must be an lvalue int64_t
      if (i->GetValue(v).IsOK())
        return v;
    }

    return 0;
  };

  // enums: get numeric value; try to resolve to the entry's name
  auto getEnumStr = [&](const char* name) -> std::string {
    if (auto* e = dynamic_cast<PvGenEnum*>(par->Get(name))) {
      int64_t val = 0;
      if (e->GetValue(val).IsOK()) {
        // Resolve enum entry name via the 2-arg API
        const PvGenEnumEntry* entry = nullptr;
        if (e->GetEntryByValue(val, &entry).IsOK() && entry != nullptr) {
          PvString s;
          if (entry->GetDisplayName(s).IsOK() && s.GetLength() > 0)
            return s.GetAscii();
          if (entry->GetName(s).IsOK() && s.GetLength() > 0)
            return s.GetAscii();
        }
        // Fallback: just print numeric value
        return std::to_string(val);
      }
    }
    return {};
  };

  const std::string vendor = getStr("DeviceVendorName");
  const std::string model = getStr("DeviceModelName");
  const std::string serial = getStr("DeviceSerialNumber");
  const std::string version = getStr("DeviceVersion");
  const int64_t width = getI64("Width");
  const int64_t height = getI64("Height");
  const std::string pixfmt = getEnumStr("PixelFormat");

  const char* proto =
      (dynamic_cast<const PvDeviceInfoGEV*>(picked) != nullptr) ? "GigE Vision"
      : (dynamic_cast<const PvDeviceInfoU3V*>(picked) != nullptr)
          ? "USB3 Vision"
          : "Unknown";

  SPDLOG_INFO("[IR CAMERA] Connected: {} {} (S/N {}) v{}", vendor, model,
              serial, version);
  SPDLOG_INFO("[IR CAMERA] Geometry: {}x{}, PixelFormat={}", width, height,
              pixfmt);
  SPDLOG_INFO("[IR CAMERA] Protocol: {}", proto);

  // 4) Optionally open stream now (GEV vs U3V handling)
  std::unique_ptr<PvStream, void (*)(PvStream*)> stream(
      PvStream::CreateAndOpen(connID, &r), [](PvStream* s) {
        if (s)
          PvStream::Free(s);
      });

  if (!r.IsOK() || !stream) {
    SPDLOG_WARN("[IR CAMERA] Stream open failed (defer to later). Code={}",
                r.GetCodeString().GetAscii());
  } else {
    if (auto* sGEV = dynamic_cast<PvStreamGEV*>(stream.get())) {
      SPDLOG_INFO("[IR CAMERA] GEV stream opened ({}:{})",
                  sGEV->GetLocalIPAddress().GetAscii(), sGEV->GetLocalPort());
      // For GEV you would later call PvDeviceGEV::SetStreamDestination(...).
    } else {
      SPDLOG_INFO("[IR CAMERA] U3V stream opened.");
    }
  }

  PvGenParameterArray* params = dev->GetParameters();

  auto setEnum = [&](const char* name, const char* val) {
    if (auto* e = dynamic_cast<PvGenEnum*>(params->Get(name))) {
      PvResult rr = e->SetValue(val);
      if (!rr.IsOK())
        SPDLOG_WARN("[IR CAMERA] Set {}={} failed: {}", name, val,
                    rr.GetCodeString().GetAscii());
    } else {
      SPDLOG_WARN("[IR CAMERA] Enum {} not found", name);
    }
  };
  auto setInt = [&](const char* name, int64_t v) {
    if (auto* i = dynamic_cast<PvGenInteger*>(params->Get(name))) {
      int64_t minV = 0, maxV = 0;
      i->GetMin(minV);
      i->GetMax(maxV);

      int64_t clamped = std::clamp<int64_t>(v, minV, maxV);
      PvResult rr = i->SetValue(clamped);
      if (!rr.IsOK()) {
        SPDLOG_WARN("[IR CAMERA] Set {}={} failed: {}", name,
                    static_cast<long long>(clamped),
                    rr.GetCodeString().GetAscii());
      }
    } else {
      SPDLOG_WARN("[IR CAMERA] Integer {} not found", name);
    }
  };

  // Turn trigger off; continuous grab
  setEnum("TriggerMode", "Off");
  setEnum("AcquisitionMode", "Continuous");

  // Set format and geometry (W=640, H=520)
  setEnum("PixelFormat", "Mono16");
  setInt("Width", 640);
  setInt("Height", 512);

  // Compute payload size (Mono8 bytes = W*H)
  uint64_t payloadSize = 640ull * 512ull;  // 332,800 bytes

  // ---------- queue buffers ----------
  std::vector<std::unique_ptr<PvBuffer>> pool;
  const uint32_t poolCount = 6;  // small pool is fine; adjust if needed
  pool.reserve(poolCount);
  for (uint32_t i = 0; i < poolCount; ++i) {
    auto b = std::make_unique<PvBuffer>();
    b->Alloc(static_cast<uint32_t>(payloadSize));
    PvResult qr = stream->QueueBuffer(b.get());
    if (!qr.IsOK())
      SPDLOG_WARN("[IR CAMERA] QueueBuffer failed: {}",
                  qr.GetCodeString().GetAscii());
    pool.emplace_back(std::move(b));
  }

  // ---------- start acquisition ----------
  if (auto* acqStart =
          dynamic_cast<PvGenCommand*>(params->Get("AcquisitionStart"))) {
    PvResult rr = acqStart->Execute();
    if (!rr.IsOK()) {
      SPDLOG_ERROR("[IR CAMERA] AcquisitionStart failed: {}",
                   rr.GetCodeString().GetAscii());
      return;
    }
  } else {
    SPDLOG_ERROR("[IR CAMERA] AcquisitionStart command not found");
    return;
  }

  // ---------- main loop: retrieve next frame, copy to std::vector<uint8_t> ----------
  uint64_t seen{};
  while (!stoken.stop_requested()) {
    ticker_.waitNext(seen);

    PvBuffer* got = nullptr;
    PvResult opRes;
    PvResult res = stream->RetrieveBuffer(&got, &opRes, 1000);  // 1s timeout
    if (!res.IsOK()) {
      SPDLOG_WARN("[IR CAMERA] RetrieveBuffer error: {}",
                  res.GetCodeString().GetAscii());
      continue;
    }

    if (!opRes.IsOK() || got == nullptr) {
      SPDLOG_WARN("[IR CAMERA] Incomplete buffer: {}",
                  opRes.GetCodeString().GetAscii());
    } else if (got->GetPayloadType() == PvPayloadTypeImage) {
      const PvImage* img = got->GetImage();
      const uint8_t* src = static_cast<const uint8_t*>(img->GetDataPointer());
      const size_t n = static_cast<size_t>(
          img->GetImageSize());  // should be 640*520 for Mono8

      // Copy to vector<uint8_t>
      std::vector<uint8_t> frame;
      frame.assign(src, src + n);

      if (!logger.enqueue(std::move(frame))) {
        SPDLOG_WARN("[IR CAMERA] RAW enqueue dropped (queue full)");
      }

      SPDLOG_INFO("[IR CAMERA] grabbed {} bytes ({}x{}, pf=Mono8)", n,
                  img->GetWidth(), img->GetHeight());
    } else {
      SPDLOG_WARN("[IR CAMERA] Non-image payload type {}",
                  static_cast<int>(got->GetPayloadType()));
    }

    // Always requeue
    if (got)
      stream->QueueBuffer(got);
  }

  // ---------- shutdown ----------
  if (auto* acqStop =
          dynamic_cast<PvGenCommand*>(params->Get("AcquisitionStop")))
    acqStop->Execute();

  stream->AbortQueuedBuffers();
  for (;;) {
    PvBuffer* leftover = nullptr;
    PvResult orr;
    if (!stream->RetrieveBuffer(&leftover, &orr, 0).IsOK() ||
        leftover == nullptr)
      break;
  }
}

void IRCamera::cleanupEbus() {
  if (m_stream) {
    if (m_device) {
      PvGenParameterArray* params = m_device->GetParameters();
      if (params) {
        PvGenCommand* stopCmd =
            dynamic_cast<PvGenCommand*>(params->Get("AcquisitionStop"));
        if (stopCmd) {
          stopCmd->Execute();
        }
      }
      m_device->StreamDisable();
    }

    m_stream->AbortQueuedBuffers();
    //Dequeue all buffers from the stream before freeing
    PvBuffer* buf = nullptr;
    PvResult opResult, result;
    while (m_stream->RetrieveBuffer(&buf, &opResult, 0).IsOK() && buf) {
      //Do not delete here, just dequeue
    }
    PvStream::Free(m_stream);
    m_stream = nullptr;
  }
}

PvBuffer* IRCamera::CaptureImageBuffer() {
  try {
    PvGenCommand* startCmd = dynamic_cast<PvGenCommand*>(
        m_device->GetParameters()->Get("AcquisitionStart"));
    if (!startCmd) {
      return nullptr;
    }
  } catch (...) {}
}

bool IRCamera::SaveBufferAsTelopsRaw(
    PvBuffer* buffer, const std::string& filename,
    const Telops::ChunkDataHeaderStruct* headerOverride) {
  if (!buffer) {
    return false;
  }

  PvImage* image = buffer->GetImage();
  if (!image || !image->GetDataPointer()) {
    return false;
  }

  uint16_t width = static_cast<uint16_t>(image->GetWidth());
  uint16_t height = static_cast<uint16_t>(image->GetHeight());
  if (width == 0 || height == 0) {
    // Invalid image dimensions
    return false;
  }
  constexpr size_t HEADER_SIZE = 256;
  size_t line_bytes = width * 2;
  size_t frame_bytes = line_bytes * (height + 2);

  std::vector<uint8_t> out(frame_bytes, 0);

  // 1. Write header at start
  if (headerOverride) {
    std::memcpy(out.data(), headerOverride, HEADER_SIZE);
  } else {
    UpdateChunkHeader();
    std::memcpy(out.data(), &m_chunkHeader, HEADER_SIZE);
  }
  // 2. (Rest of first two lines is already zero due to vector init)
  // 3. Write image data after first two lines
  std::memcpy(out.data() + 2 * line_bytes, image->GetDataPointer(),
              static_cast<size_t>(width) * static_cast<size_t>(height) * 2);

  std::ofstream file(filename, std::ios::binary);
  if (!file.is_open())
    return false;

  auto t_write_start = std::chrono::steady_clock::now();
  file.write(reinterpret_cast<const char*>(out.data()),
             static_cast<std::streamsize>(frame_bytes));
  auto t_write_end = std::chrono::steady_clock::now();
  auto d = t_write_end - t_write_start;
  std::cout
      << "[Telops] SD write time: "
      << static_cast<double>(
             std::chrono::duration_cast<std::chrono::microseconds>(d).count()) /
             1000.0
      << " ms" << std::endl;

  return file.good();
}

void IRCamera::UpdateChunkHeader() {
  // if (!m_initialized) {
  //   return;
  // }

  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
                          now.time_since_epoch()) %
                      1000000;

  m_chunkHeader.ChunkFrameIDValue++;
  m_chunkHeader.ChunkPOSIXTimeValue = static_cast<uint32_t>(time_t);
  m_chunkHeader.ChunkSubSecondTimeValue =
      static_cast<uint32_t>(microseconds.count() * 10);

  //Use cached temperature values from the temperature thread
  m_chunkHeader.ChunkTemperatureSensorValue =
      static_cast<int16_t>(m_cachedFpaTemperature * 100.0f);
  m_chunkHeader.ChunkTemperatureInternalLensValue =
      static_cast<int16_t>(m_cachedLensTemperature * 100.0f);

  if (m_operationalCommandWrite) {
    uint64_t integrationTime = m_operationalCommandWrite->GetIntegrationTime();
    m_chunkHeader.ChunkExposureTimeValue =
        static_cast<uint32_t>(integrationTime * 100);
    uint64_t frameTime = m_operationalCommandWrite->GetFrametime();
    float frameRateHz = 1000000.0f / static_cast<float>(frameTime);
    m_chunkHeader.ChunkAcquisitionFrameRateValue =
        static_cast<uint32_t>(frameRateHz * 1000.0f);
  }
}

}  // namespace sober::camera