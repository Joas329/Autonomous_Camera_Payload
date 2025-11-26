#include "camera/OpticalCamera.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stop_token>
#include <thread>
#include <vector>

#include <SpinGenApi/SpinnakerGenApi.h>
#include <Spinnaker.h>
#include <lz4.h>
#include <spdlog/spdlog.h>
#include <turbojpeg.h>

#include "LastFrame.hpp"
#include "logger/ImageLogger.hpp"

namespace fs = std::filesystem;
using namespace Spinnaker;
using namespace Spinnaker::GenApi;

namespace sober::camera {

static bool encodeJpegMono8(const unsigned char* src, int width, int height,
                            int stride, int quality,
                            std::vector<uint8_t>& out) {
  tjhandle tj = tjInitCompress();
  if (!tj)
    return false;

  unsigned char* jpegBuf = nullptr;
  unsigned long jpegSize = 0;

  const int rc =
      tjCompress2(tj, src, width, stride, height, TJPF_GRAY, &jpegBuf,
                  &jpegSize, TJSAMP_GRAY, quality, TJFLAG_FASTDCT);

  const bool ok = (rc == 0 && jpegBuf && jpegSize > 0);
  if (ok)
    out.assign(jpegBuf, jpegBuf + jpegSize);

  if (jpegBuf)
    tjFree(jpegBuf);
  tjDestroy(tj);
  return ok;
}

static bool writeBytes(const std::filesystem::path& path,
                       const std::vector<uint8_t>& data) {
  std::ofstream ofs(path, std::ios::binary);
  if (!ofs)
    return false;
  ofs.write(reinterpret_cast<const char*>(data.data()),
            static_cast<std::streamsize>(data.size()));
  return ofs.good();
}

std::vector<std::uint8_t> lz4Compress(const std::vector<std::uint8_t>& src) {
  const int srcSize = static_cast<int>(src.size());
  const int maxDstSize = LZ4_compressBound(srcSize);
  std::vector<std::uint8_t> dst(static_cast<std::size_t>(maxDstSize));

  const int compressedSize = LZ4_compress_default(
      reinterpret_cast<const char*>(src.data()),
      reinterpret_cast<char*>(dst.data()), srcSize, maxDstSize);

  if (compressedSize <= 0) {
    throw std::runtime_error("LZ4 compression failed");
  }

  dst.resize(static_cast<std::size_t>(compressedSize));
  return dst;
}

bool OpticalCamera::start() {
  std::cout<<"Camera entry perchance."<< std::endl;
  if (opticalThread_.joinable()) {
    return false;
  }

  opticalThread_ = std::jthread([this](std::stop_token st) { run(st); });
  return true;
}

void OpticalCamera::stop() {
  if (!opticalThread_.joinable()) {
    return;
  }
  opticalThread_.request_stop();
  opticalThread_.join();
}

static void set_integer(INodeMap& nm, const char* node, int64_t value) {
  try {
    CIntegerPtr i = nm.GetNode(node);
    if (!IsWritable(i))
      return;
    if (value < i->GetMin())
      value = i->GetMin();
    if (value > i->GetMax())
      value = i->GetMax();
    i->SetValue(value);
  } catch (...) {}
}

static void set_enum_entry(INodeMap& nm, const char* node, const char* entry) {
  try {
    CEnumerationPtr e = nm.GetNode(node);
    if (!IsReadable(e) || !IsWritable(e)) {
      return;
    }
    CEnumEntryPtr v = e->GetEntryByName(entry);
    if (!IsReadable(v)) {
      return;
    }
    e->SetIntValue(v->GetValue());
  } catch (...) {}
}

static void set_bool(INodeMap& nm, const char* node, bool value) {
  try {
    CBooleanPtr b = nm.GetNode(node);
    if (!IsWritable(b)) {
      return;
    }
    b->SetValue(value);
  } catch (...) {}
}

static void set_float(INodeMap& nm, const char* node, double value) {
  try {
    CFloatPtr f = nm.GetNode(node);
    if (!IsWritable(f)) {
      return;
    }
    const double minv = f->GetMin();
    const double maxv = f->GetMax();
    if (value < minv) {
      value = minv;
    }
    if (value > maxv) {
      value = maxv;
    }
    f->SetValue(value);
  } catch (...) {}
}

static std::string make_filename() {
  using clock = std::chrono::system_clock;
  const auto now = clock::now();
  const auto t = clock::to_time_t(now);
  const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) %
                  1000;

  std::tm tm{};
#if defined(_POSIX_VERSION)
  localtime_r(&t, &tm);
#else
  tm = *std::localtime(&t);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S") << '_' << std::setw(3)
      << std::setfill('0') << ms.count();
  return oss.str();
}

static inline std::string to_std(const Spinnaker::GenICam::gcstring& s) {
  return std::string(s.c_str());
}
static inline std::string read_str(const Spinnaker::GenApi::CStringPtr& n) {
  return (IsReadable(n) ? to_std(n->ToString()) : std::string("?"));
}

static void initCameraParams(Spinnaker::GenApi::INodeMap& nodeMap,
                             Spinnaker::GenApi::INodeMap& sNodeMap,
                             Spinnaker::GenApi::INodeMap& tlDev, double fps) {
  try {
    if (auto m = CBooleanPtr(nodeMap.GetNode("DeviceLinkThroughputLimitMode"));
        IsWritable(m)) {
      m->SetValue(true);
    }
    if (auto l = CIntegerPtr(nodeMap.GetNode("DeviceLinkThroughputLimit"));
        IsWritable(l)) {
      const int64_t cap = 200000000;  // ~200 Mbps safety cap
      l->SetValue(std::min<int64_t>(cap, l->GetMax()));
      SPDLOG_INFO("Device(DeviceLinkThroughputLimit) -> {}",
                  (long long)l->GetValue());
    }

    set_bool(nodeMap, "AcquisitionFrameRateEnable", true);
    set_float(nodeMap, "AcquisitionFrameRate", fps);

    if (auto cntMode =
            CEnumerationPtr(sNodeMap.GetNode("StreamBufferCountMode"));
        IsWritable(cntMode)) {
      if (auto manual = CEnumEntryPtr(cntMode->GetEntryByName("Manual"));
          IsReadable(manual)) {
        cntMode->SetIntValue(manual->GetValue());
      }
    }
    set_integer(sNodeMap, "StreamBufferCountManual", 96);
    set_integer(sNodeMap, "StreamTransferSize", 1048576);
    set_integer(sNodeMap, "StreamTransferNumberUrb", 64);
    if (auto bh = CEnumerationPtr(sNodeMap.GetNode("StreamBufferHandlingMode"));
        IsReadable(bh) && IsWritable(bh)) {
      if (auto newestOnly = CEnumEntryPtr(bh->GetEntryByName("NewestOnly"));
          IsReadable(newestOnly)) {
        bh->SetIntValue(newestOnly->GetValue());
      }
    }

    set_enum_entry(nodeMap, "TriggerMode", "Off");  // free-run
    set_enum_entry(nodeMap, "AcquisitionMode", "Continuous");
    set_enum_entry(nodeMap, "PixelFormat", "Mono8");
    set_enum_entry(nodeMap, "ExposureAuto", "Off");
    set_enum_entry(nodeMap, "ExposureMode", "Timed");
    set_float(nodeMap, "ExposureTime", 8000.0);  // us

    try {
      if (auto p =
              Spinnaker::GenApi::CFloatPtr(tlDev.GetNode("DeviceLinkSpeed"));
          IsReadable(p)) {
        SPDLOG_INFO("DeviceLinkSpeed(Mbps) = {}", p->GetValue());
      }
    } catch (...) {}
    try {
      if (auto p = Spinnaker::GenApi::CIntegerPtr(
              sNodeMap.GetNode("StreamBufferCountResult"));
          IsReadable(p)) {
        SPDLOG_INFO("StreamBufferCountResult={}", (long long)p->GetValue());
      }
    } catch (...) {}
  } catch (...) {
    SPDLOG_WARN(
        "initCameraParams: encountered an exception; continuing with "
        "best-effort defaults");
  }
}

void OpticalCamera::pushJpegToBuffer(std::vector<uint8_t> jpg, size_t w,
                                     size_t h, size_t stride,
                                     uint8_t channels) {
  {
    std::lock_guard<std::mutex> lk(bufferMutex_);
    lastImgae_ = std::move(jpg);
    SPDLOG_INFO("[OPTICAL CAMERA] Image has size: {}", lastImgae_.size());
    const std::size_t bufIdx =
        current_idx.fetch_add(1, std::memory_order_acq_rel);
    buffers[bufIdx % buffers.size()] = lastImgae_;

    originalSize.store(static_cast<uint32_t>(lastImgae_.size()),
                       std::memory_order_release);
    last_w_ = w;
    last_h_ = h;
    last_stride_ = stride;
    last_channels_ = channels;
  }
}

void OpticalCamera::run(std::stop_token stoken) {
  static constexpr double kFPS = 1.0;
  const fs::path kOutputDir = "/home/sober/Autonomous_Control/images";

  imglog::ImageLogger<64> logger(imglog::FlushPolicy::Never);
  logger.start();

  try {
    SPDLOG_INFO("[OPTICAL CAMERA] CWD: {}", fs::current_path().string());
    fs::create_directories(kOutputDir);
    SPDLOG_INFO("[OPTICAL CAMERA] Output dir ready: {}", kOutputDir.string());
  } catch (...) {}

  SystemPtr system;
  CameraList camList;
  CameraPtr cam;
  bool started_acq = false;

  try {
    system = System::GetInstance();
    camList = system->GetCameras();
    if (camList.GetSize() == 0) {
      SPDLOG_ERROR("No camera... EXIT");
      camList.Clear();
      system->ReleaseInstance();
      return;
    }

    for (unsigned i = 0; i < camList.GetSize(); ++i) {
      CameraPtr c = camList.GetByIndex(i);
      auto& tl = c->GetTLDeviceNodeMap();
      auto model = Spinnaker::GenApi::CStringPtr(tl.GetNode("DeviceModelName"));
      if (IsReadable(model) &&
          std::string(model->ToString()) == "BFS-U3-122S6C") {
        cam = c;
        break;
      }
    }
    if (!cam) {
      cam = camList.GetByIndex(0);
    }

    cam->Init();

    auto& nodeMap = cam->GetNodeMap();           // device features
    auto& tlDev = cam->GetTLDeviceNodeMap();     // TL device
    auto& sNodeMap = cam->GetTLStreamNodeMap();  // TL stream

    initCameraParams(nodeMap, sNodeMap, tlDev, kFPS);

    cam->BeginAcquisition();
    started_acq = true;

    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::milliseconds(
        static_cast<int>(std::llround(1000.0 / kFPS)));
    // auto next_tick = clock::now() + period;

    int consec_incomplete = 0;

    uint64_t seen{0};
    auto ms = [](auto dt) {
      return std::chrono::duration_cast<std::chrono::milliseconds>(dt).count();
    };
    auto us = [](auto dt) {
      return std::chrono::duration_cast<std::chrono::microseconds>(dt).count();
    };
    while (!stoken.stop_requested()) {
      ticker_.waitNext(seen);

      const auto t_loop_start = clock::now();
      const auto t_cap_start = clock::now();

      ImagePtr raw;
      try {
        const unsigned timeout_ms = static_cast<unsigned>(
            std::max<int>(static_cast<int>(period.count()) + 250, 1000));
        raw = cam->GetNextImage(timeout_ms);
      } catch (const std::exception& e) {
        SPDLOG_WARN("[OPTICAL CAMERA] GetNextImage timeout/err: {}", e.what());
        continue;
      } catch (...) {
        SPDLOG_WARN("[OPTICAL CAMERA] GetNextImage timeout/err (unknown)");
        continue;
      }

      const auto t_cap_end = clock::now();

      // const auto t_acq_end = clock::now();

      if (!raw || raw->IsIncomplete()) {
        ++consec_incomplete;
        if (raw) {
          SPDLOG_WARN(
              "[OPTICAL CAMERA] Incomplete frame: status {} (streak={})",
              static_cast<int>(raw->GetImageStatus()), consec_incomplete);
          try {
            raw->Release();
          } catch (...) {}
        }
        if (consec_incomplete >= 10) {
          SPDLOG_WARN(
              "[OPTICAL CAMERA] Too many incompletes; restarting acquisition");
          try {
            if (started_acq)
              cam->EndAcquisition();
          } catch (...) {}
          std::this_thread::sleep_for(std::chrono::milliseconds(50));
          try {
            cam->BeginAcquisition();
            started_acq = true;
          } catch (...) {}
          consec_incomplete = 0;
        }
        continue;
      }
      consec_incomplete = 0;

      ImagePtr mono = raw;
      if (raw->GetPixelFormat() != PixelFormat_Mono8) {
        try {
          ImageProcessor proc;
          proc.SetColorProcessing(
              SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
          mono = proc.Convert(raw, PixelFormat_Mono8);
        } catch (...) {
          SPDLOG_WARN("[OPTICAL CAMERA] Convert to Mono8 failed");
          try {
            raw->Release();
          } catch (...) {}
          continue;
        }
      }

      const size_t w = mono->GetWidth();
      const size_t h = mono->GetHeight();
      const size_t stride = mono->GetStride();
      const size_t raw_sz = stride * h;

      std::filesystem::path savedPath;
      const auto t_enq_start = clock::now();
      {
        const unsigned char* src_raw =
            static_cast<const unsigned char*>(mono->GetData());
        std::vector<uint8_t> rawBytes(raw_sz);
        std::memcpy(rawBytes.data(), src_raw, raw_sz);

        if (!logger.enqueue(std::move(rawBytes))) {
          SPDLOG_WARN("[OPTICAL CAMERA] RAW enqueue dropped (queue full)");
        }
      }
      const auto t_enq_end = clock::now();

      // std::vector<uint8_t> jpgBytes;

      // const unsigned char* src =
      //     static_cast<const unsigned char*>(mono->GetData());
      // const auto t_enc_start = clock::now();
      // bool enc_ok =
      //     encodeJpegMono8(src, static_cast<int>(w), static_cast<int>(h),
      //                     static_cast<int>(stride), 85, jpgBytes);

      // const auto t_enc_end = clock::now();

      // SPDLOG_INFO("[OPTICAL CAMERA] JPG Size: {}", jpgBytes.size());

      // if (enc_ok) {
      //   const std::string stem = make_filename();
      //   savedPath = kOutputDir / std::filesystem::path(stem).filename();
      //   savedPath.replace_extension(".jpg");

      //   bool wrote_ok = writeBytes(savedPath, jpgBytes);
      //   const auto t_io_end = clock::now();

      //   SPDLOG_INFO("[OPTICAL CAMERA] encode={}ms write={}ms path={}",
      //               std::chrono::duration_cast<std::chrono::milliseconds>(
      //                   t_enc_end - t_enc_start)
      //                   .count(),
      //               std::chrono::duration_cast<std::chrono::milliseconds>(
      //                   t_io_end - t_enc_start)
      //                   .count(),
      //               wrote_ok ? savedPath.string() : "(failed)");

      //   pushJpegToBuffer(std::move(jpgBytes), w, h, stride, /*channels*/ 1);
      // } else {
      //   SPDLOG_WARN("[OPTICAL CAMERA] JPEG encode failed");
      // }

      const auto t_enc_start = clock::now();
      auto t_enc_end = clock::now();
      {
        std::vector<uint8_t> jpgBytes;
        const unsigned char* src =
            static_cast<const unsigned char*>(mono->GetData());
        const bool enc_ok = encodeJpegMono8(src, int(w), int(h), int(stride),
                                            /*quality*/ 85, jpgBytes);
        t_enc_end = clock::now();

        if (enc_ok) {
          // you already have this function
          pushJpegToBuffer(std::move(jpgBytes), w, h, stride, /*channels*/ 1);
        } else {
          SPDLOG_WARN("[OPTICAL CAMERA] JPEG encode failed");
        }
      }

      try {
        raw->Release();
      } catch (...) {}
      const auto t_loop_end = clock::now();
      const auto loop_ms = ms(t_loop_end - t_loop_start);
      const auto cap_ms = ms(t_cap_end - t_cap_start);
      const auto enq_us = us(t_enq_end - t_enq_start);  // higher resolution
      const auto enc_us = us(t_enc_end - t_enc_start);

      SPDLOG_INFO(
          "[OPTICAL CAMERA] loop={}ms capture={}ms enqueue={}us raw={}B "
          "encode={}us",
          loop_ms, cap_ms, enq_us, raw_sz, enc_us);
    }
  } catch (...) {
    SPDLOG_ERROR("[OPTICAL CAMERA] Fatal error in run()");
  }
  try {
    if (started_acq && cam)
      cam->EndAcquisition();
  } catch (...) {}
  try {
    if (cam)
      cam->DeInit();
  } catch (...) {}
  try {
    cam = nullptr;
  } catch (...) {}
  try {
    camList.Clear();
  } catch (...) {}
  try {
    if (system)
      system->ReleaseInstance();
  } catch (...) {}
}
}  // namespace sober::camera
