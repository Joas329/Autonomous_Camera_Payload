// *****************************************************************************
//
//     Copyright (c) 2025, York Space Systems, All rights reserved.
//     FLIR Blackfly S BFS-U3-122S6C-C RGB camera driver
//     Author: Joaquin Philco
//
// *****************************************************************************

#include <chrono>
#include <vector>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <stdexcept>
#include <filesystem>
#include <turbojpeg.h>
#include <Spinnaker.h>
#include <spdlog/spdlog.h>
#include "camera/Ticker.hpp"
#include "eventbus/Event.hpp"
#include "logger/ImageLogger.hpp"
#include "camera/OpticalSensor.hpp"
#include <SpinGenApi/SpinnakerGenApi.h>

using Spinnaker::System;
using Spinnaker::ImagePtr;
using Spinnaker::CameraList;
using Spinnaker::GenApi::CFloatPtr;
using Spinnaker::GenApi::IsWritable;
using Spinnaker::GenApi::IsReadable;
using Spinnaker::GenApi::IsAvailable;
using Spinnaker::GenApi::CEnumEntryPtr;
using Spinnaker::GenApi::CEnumerationPtr;

using namespace Spinnaker;
using namespace Spinnaker::GenApi;

namespace fs = std::filesystem;

static std::string make_stem_from_image_timestamp(
    const Spinnaker::ImagePtr& img,
    int frame_count)
{
    // Fallback: if chunk timestamp isn't available, still produce something unique.
    uint64_t ts = 0;
    bool have_ts = false;

    try {
        // Requires ChunkTimestamp enabled.
        const auto cd = img->GetChunkData();
        ts = cd.GetTimestamp();        // typically in nanoseconds (camera timebase)
        have_ts = true;
    } catch (...) {
        have_ts = false;
    }

    std::ostringstream oss;

    if (have_ts) {
        // Represent as seconds + nanoseconds remainder for readability.
        const uint64_t sec = ts / 1000000000ULL;
        const uint64_t nsec = ts % 1000000000ULL;

        oss << "frame_camts_"
            << sec << "_"
            << std::setw(9) << std::setfill('0') << nsec
            << "_" << std::setw(6) << std::setfill('0') << frame_count;
    } else {
        // Fallback to wall-clock if chunk isn't available
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);

        std::tm tm{};
    #if defined(_POSIX_VERSION)
        localtime_r(&t, &tm);
    #else
        tm = *std::localtime(&t);
    #endif

        oss << "frame_hostts_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S")
            << "_" << std::setw(6) << std::setfill('0') << frame_count;
    }

    return oss.str();
}

static bool encodeJpegMono8(
    const unsigned char* src,
    int width,
    int height,
    int stride,
    int quality,
    std::vector<uint8_t>& out)
{
    tjhandle tj = tjInitCompress();
    if (!tj) return false;

    unsigned char* jpegBuf = nullptr;
    unsigned long jpegSize = 0;

    const int rc = tjCompress2(
        tj,
        src,
        width,
        stride,
        height,
        TJPF_GRAY,
        &jpegBuf,
        &jpegSize,
        TJSAMP_GRAY,
        quality,
        TJFLAG_FASTDCT
    );

    const bool ok = (rc == 0 && jpegBuf && jpegSize > 0);
    if (ok) out.assign(jpegBuf, jpegBuf + jpegSize);

    if (jpegBuf) tjFree(jpegBuf);
    tjDestroy(tj);
    return ok;
}

static bool writeBytes(const std::filesystem::path& path,
                       const std::vector<uint8_t>& data)
{
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) return false;
    ofs.write(reinterpret_cast<const char*>(data.data()),
              static_cast<std::streamsize>(data.size()));
    return ofs.good();
}

// Converts img -> Mono8 (if needed), JPEG encodes, saves JPEG.
// NOTE: does NOT modify/release the input img; caller still owns img->Release().
static bool save_jpeg_from_image(
    const Spinnaker::ImagePtr& img,
    const std::filesystem::path& out_dir,
    const std::string& stem,
    int jpeg_quality,
    std::string* err_out = nullptr)
{
    try {
        if (!img) {
            if (err_out) *err_out = "img is null";
            return false;
        }

        Spinnaker::ImagePtr mono = img;

        // Ensure Mono8 for your turbojpeg path
        if (img->GetPixelFormat() != Spinnaker::PixelFormat_Mono8) {
            Spinnaker::ImageProcessor proc;
            proc.SetColorProcessing(SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR);
            mono = proc.Convert(img, Spinnaker::PixelFormat_Mono8);
        }

        const int w = static_cast<int>(mono->GetWidth());
        const int h = static_cast<int>(mono->GetHeight());
        const int stride = static_cast<int>(mono->GetStride());

        const unsigned char* src =
            static_cast<const unsigned char*>(mono->GetData());

        std::vector<uint8_t> jpgBytes;
        if (!encodeJpegMono8(src, w, h, stride, jpeg_quality, jpgBytes)) {
            if (err_out) *err_out = "encodeJpegMono8 failed";
            return false;
        }

        const std::filesystem::path jpg_path = out_dir / (stem + ".jpg");
        if (!writeBytes(jpg_path, jpgBytes)) {
            if (err_out) *err_out = "writeBytes failed for " + jpg_path.string();
            return false;
        }

        SPDLOG_INFO("[OPTICAL] Saved JPEG: {} ({} bytes)", jpg_path.string(), jpgBytes.size());
        return true;

    } catch (const std::exception& e) {
        if (err_out) *err_out = e.what();
        return false;
    } catch (...) {
        if (err_out) *err_out = "unknown exception";
        return false;
    }
}

namespace sober::camera {
    FLIR_Blackfly_S::FLIR_Blackfly_S(Ticker& tick)
        : m_running(false),
          m_output_dir("/home/sober/Autonomous_Control/images"),
          m_default_exposure_us(100000),
          m_min_exposure_us(1.0),
          m_max_exposure_us(1000000.0),
          m_trigger_mode_enabled(false),
          m_ticker(tick),
          m_system(SystemPtr()),
          m_cam_list(CameraList()),
          m_cam(CameraPtr())
    {
        const fs::path kOutputDir = m_output_dir;

        imglog::ImageLogger<64> logger(imglog::FlushPolicy::Never);
        logger.start();
        try {
            SPDLOG_INFO("[OPTICAL CAMERA] CWD: {}", fs::current_path().string());
            fs::create_directories(kOutputDir);
            SPDLOG_INFO("[OPTICAL CAMERA] Output dir ready: {}", kOutputDir.string());
        } catch (...) {}
    }

    FLIR_Blackfly_S::~FLIR_Blackfly_S() {
        // stop();

        if (m_cam) {
            m_cam->DeInit();
            m_cam = nullptr;
        }

        m_system->ReleaseInstance();
    }

    bool FLIR_Blackfly_S::findFLIRCamera()
    {
        m_system = System::GetInstance();
        m_cam_list = m_system->GetCameras();

        SPDLOG_INFO("[FLIR] GetCameras() size={}", m_cam_list.GetSize());

        if (m_cam_list.GetSize() == 0)
        {
            SPDLOG_CRITICAL("No cameras found!");
            return false;
        }

        for (unsigned i = 0; i < m_cam_list.GetSize(); ++i)
        {
            CameraPtr c = m_cam_list.GetByIndex(i);
            auto& tl = c->GetTLDeviceNodeMap();
            auto model = Spinnaker::GenApi::CStringPtr(tl.GetNode("DeviceModelName"));

            if (IsReadable(model) && std::string(model->ToString()) == FLIR_BLACKFLY_S_ID) {
                m_cam = c;
                SPDLOG_INFO("FLIR Blackfly S camera found!");
                return true;
            }
        }

        std::cout << "FLIR Blackfly S camera not found, using first available camera." << std::endl;

        return false;
    }

    bool FLIR_Blackfly_S::start() 
    {
        if (m_optical_thread.joinable())
        {
            std::cout << "Camera thread already running." << std::endl;
            return false;
        }

        m_optical_thread = std::jthread([this] (std::stop_token st) { main_camera_thread(st);});
        return true;
    }

    void FLIR_Blackfly_S::configureTriggerMode(bool enable)
    {
        if (!m_cam)
        {
            throw std::runtime_error("Camera not initialized");
        }

        auto& nodemap = m_cam->GetNodeMap();
        // Configure trigger mode
        CEnumerationPtr trigger_mode_ptr = nodemap.GetNode("TriggerMode");
        if (IsAvailable(trigger_mode_ptr) && IsWritable(trigger_mode_ptr))
        {
            CEnumEntryPtr off_entry = trigger_mode_ptr->GetEntryByName("Off");
            CEnumEntryPtr on_entry = trigger_mode_ptr->GetEntryByName("On");
            if (IsAvailable(off_entry) && IsReadable(off_entry) &&
                IsAvailable(on_entry) && IsReadable(on_entry)) {
                if (enable)
                {
                    trigger_mode_ptr->SetIntValue(on_entry->GetValue());
                    std::cout << "Trigger mode enabled" << std::endl;
                } else
                {
                    trigger_mode_ptr->SetIntValue(off_entry->GetValue());
                    std::cout << "Trigger mode disabled" << std::endl;
                }
            }
        }

        // Configure trigger source
        CEnumerationPtr trigger_source_ptr = nodemap.GetNode("TriggerSource");
        if (IsAvailable(trigger_source_ptr) && IsWritable(trigger_source_ptr))
        {
            CEnumEntryPtr software_entry = trigger_source_ptr->GetEntryByName("Software");
            if (IsAvailable(software_entry) && IsReadable(software_entry))
            {
                trigger_source_ptr->SetIntValue(software_entry->GetValue());
                std::cout << "Trigger source set to software" << std::endl;
            }
        }

        // Configure trigger selector
        CEnumerationPtr trigger_selector_ptr = nodemap.GetNode("TriggerSelector");
        if (IsAvailable(trigger_selector_ptr) && IsWritable(trigger_selector_ptr))
        {
            CEnumEntryPtr frame_start_entry = trigger_selector_ptr->GetEntryByName("FrameStart");
            if (IsAvailable(frame_start_entry) && IsReadable(frame_start_entry))
            {
                trigger_selector_ptr->SetIntValue(frame_start_entry->GetValue());
                std::cout << "Trigger selector set to FrameStart" << std::endl;
            }
        }

        // // Set minimum trigger delay (29 microseconds)
        // CFloatPtr trigger_delay_ptr = nodemap.GetNode("TriggerDelay");
        // if (IsAvailable(trigger_delay_ptr) && IsWritable(trigger_delay_ptr))
        // {
        //     trigger_delay_ptr->SetValue(29.0);
        //     std::cout << "Trigger delay set to 29 microseconds" << std::endl;
        // }
    }

    void FLIR_Blackfly_S::setup_camera()
    {
        if (!m_cam) {
            SPDLOG_ERROR("[OPTICAL] setup_camera(): m_cam is null");
            return;
        }

        // ========= Device/Stream node maps =========
        auto& nodemap = m_cam->GetNodeMap();
        auto& sNodeMap = m_cam->GetTLStreamNodeMap();      // stream/buffer settings
        auto& tlDev = m_cam->GetTLDeviceNodeMap();         // device transport info

        // ========= 0) Transport throughput cap (important on embedded/USB) =========
        if (auto m = CBooleanPtr(nodemap.GetNode("DeviceLinkThroughputLimitMode"));
        IsWritable(m)) 
        {
            m->SetValue(true);
        }
        if (auto l = CIntegerPtr(nodemap.GetNode("DeviceLinkThroughputLimit")); IsWritable(l))
        {
            const int64_t cap = 200000000;  // ~200 Mbps safety cap
            l->SetValue(std::min<int64_t>(cap, l->GetMax()));
            SPDLOG_INFO("Device(DeviceLinkThroughputLimit) -> {}", (long long)l->GetValue());
        }

        // Set acquisition mode to continuous
        CEnumerationPtr acq_mode_ptr = nodemap.GetNode("AcquisitionMode");
        if (IsAvailable(acq_mode_ptr) && IsWritable(acq_mode_ptr)) {
            CEnumEntryPtr continuous_entry = acq_mode_ptr->GetEntryByName("Continuous");
            if (IsAvailable(continuous_entry) && IsReadable(continuous_entry)) {
            acq_mode_ptr->SetIntValue(continuous_entry->GetValue());
            }
        }

        // Set exposure to manual
        CEnumerationPtr exp_auto_ptr = nodemap.GetNode("ExposureAuto");
        if (IsAvailable(exp_auto_ptr) && IsWritable(exp_auto_ptr)) {
            CEnumEntryPtr off_entry = exp_auto_ptr->GetEntryByName("Off");
            if (IsAvailable(off_entry) && IsReadable(off_entry)) {
            exp_auto_ptr->SetIntValue(off_entry->GetValue());
            std::cout << "Exposure set to manual mode" << std::endl;
            }
        }

        // Set exposure time
        CFloatPtr exp_time_ptr = nodemap.GetNode("ExposureTime");
        if (IsAvailable(exp_time_ptr) && IsWritable(exp_time_ptr)) {
            exp_time_ptr->SetValue(m_default_exposure_us);
            std::cout << "Initial exposure time set to " << m_default_exposure_us << " microseconds" << std::endl;
        }

        // Configure trigger mode (disabled by default)
        configureTriggerMode(false);

        // Disable AcquisitionFrameRateAuto
        CEnumerationPtr frame_rate_auto_ptr = nodemap.GetNode("AcquisitionFrameRateAuto");
        if (IsAvailable(frame_rate_auto_ptr) && IsWritable(frame_rate_auto_ptr)) {
            CEnumEntryPtr off_entry = frame_rate_auto_ptr->GetEntryByName("Off");
            if (IsAvailable(off_entry) && IsReadable(off_entry)) {
            frame_rate_auto_ptr->SetIntValue(off_entry->GetValue());
            std::cout << "AcquisitionFrameRateAuto disabled" << std::endl;
            }
        }

        // Enable frame rate control
        CEnumerationPtr frame_rate_enable_ptr = nodemap.GetNode("AcquisitionFrameRateEnable");
        if (IsAvailable(frame_rate_enable_ptr) && IsWritable(frame_rate_enable_ptr)) {
            CEnumEntryPtr on_entry = frame_rate_enable_ptr->GetEntryByName("On");
            if (IsAvailable(on_entry) && IsReadable(on_entry)) {
            frame_rate_enable_ptr->SetIntValue(on_entry->GetValue());
            std::cout << "Frame rate control enabled" << std::endl;
            }
        }

        // Enable frame rate persistence
        Spinnaker::GenApi::CBooleanPtr frame_rate_persistence_ptr = nodemap.GetNode("AcquisitionFrameRatePersistence");
        if (IsAvailable(frame_rate_persistence_ptr) && IsWritable(frame_rate_persistence_ptr)) {
            frame_rate_persistence_ptr->SetValue(true);
            std::cout << "Frame rate persistence enabled" << std::endl;
        }

        // Get frame rate limits and set frame rate
        CFloatPtr frame_rate_ptr = nodemap.GetNode("AcquisitionFrameRate");
        if (IsAvailable(frame_rate_ptr) && IsReadable(frame_rate_ptr)) {
            double min_fps = frame_rate_ptr->GetMin();
            double max_fps = frame_rate_ptr->GetMax();
            std::cout << "Frame rate limits: " << min_fps << " to " << max_fps << " fps" << std::endl;

            // Set frame rate to maximum since we'll throttle in software
            if (IsAvailable(frame_rate_ptr) && IsWritable(frame_rate_ptr)) {
            frame_rate_ptr->SetValue(max_fps);
            std::cout << "Frame rate set to maximum " << max_fps << "fps (will throttle to 9fps in software)" << std::endl;
            }
        }

        // ========= 5) Pixel format (prefer Mono8 for stability) =========
        {
            CEnumerationPtr pixel_format_ptr = nodemap.GetNode("PixelFormat");
            if (IsAvailable(pixel_format_ptr) && IsWritable(pixel_format_ptr)) {
                // Force Mono8 first (known-good). You can later switch to BayerRG8/RGB8.
                CEnumEntryPtr mono8 = pixel_format_ptr->GetEntryByName("Mono8");
                if (IsAvailable(mono8) && IsReadable(mono8)) {
                    pixel_format_ptr->SetIntValue(mono8->GetValue());
                    SPDLOG_INFO("[OPTICAL] PixelFormat=Mono8");
                } else {
                    SPDLOG_WARN("[OPTICAL] Mono8 not available; leaving PixelFormat unchanged");
                }
            }
        }

        {
            CBooleanPtr chunkModeActive = nodemap.GetNode("ChunkModeActive");
            if (IsAvailable(chunkModeActive) && IsWritable(chunkModeActive))
            {
                chunkModeActive->SetValue(true);
                SPDLOG_INFO("[OPTICAL] ChunkModeActive=On");
            } else
            {
                SPDLOG_WARN("[OPTICAL] ChunkModeActive not available/writable");
            }

            CEnumerationPtr chunkSelector = nodemap.GetNode("ChunkSelector");
            if (IsAvailable(chunkSelector) && IsWritable(chunkSelector))
            {
                CEnumEntryPtr tsEntry = chunkSelector->GetEntryByName("Timestamp");
                if (IsAvailable(tsEntry) && IsReadable(tsEntry))
                {
                    chunkSelector->SetIntValue(tsEntry->GetValue());

                    CBooleanPtr chunkEnable = nodemap.GetNode("ChunkEnable");
                    if (IsAvailable(chunkEnable) && IsWritable(chunkEnable)) {
                        chunkEnable->SetValue(true);
                        SPDLOG_INFO("[OPTICAL] ChunkTimestamp enabled");
                    } else
                    {
                        SPDLOG_WARN("[OPTICAL] ChunkEnable not available/writable");
                    }
                } else
                {
                    SPDLOG_WARN("[OPTICAL] ChunkSelector 'Timestamp' not available/readable");
                }
            } else
            {
                SPDLOG_WARN("[OPTICAL] ChunkSelector not available/writable");
            }
        }
    }

    void FLIR_Blackfly_S::main_camera_thread(std::stop_token st)
    {
        SPDLOG_INFO("[OPTICAL] Camera thread starting...");

        // ---- Setup once (experiment start) ----
        if (!findFLIRCamera() || !m_cam) {
            SPDLOG_ERROR("[OPTICAL] No camera found; thread exiting");
            m_state.store(CamState::Shutdown);
            return;
        }

        try {
            SPDLOG_INFO("[OPTICAL] Init camera...");
            m_cam->Init();

            SPDLOG_INFO("[OPTICAL] Applying default parameters...");
            setup_camera();

            m_state.store(CamState::Idle);
            SPDLOG_INFO("[OPTICAL] Camera configured; state=Idle (waiting for start)");
        } catch (const std::exception& e) {
            SPDLOG_ERROR("[OPTICAL] Setup failed: {}", e.what());
            m_state.store(CamState::Shutdown);
            return;
        }

        // ---- Command loop: Idle <-> Acquiring until Shutdown ----
        while (!st.stop_requested())
        {
            // 1) Wait until we should acquire or shutdown
            {
                std::unique_lock<std::mutex> lk(m_mtx);
                m_conditionVariable.wait(lk, [&] {
                    const auto s = m_state.load();
                    return st.stop_requested()
                        || s == CamState::Acquiring
                        || s == CamState::Shutdown;
                });
            }

            const auto state_now = m_state.load();
            if (st.stop_requested() || state_now == CamState::Shutdown) {
                SPDLOG_INFO("[OPTICAL] Stop requested; leaving thread main loop");
                break;
            }

            // 2) Start acquisition when commanded
            if (m_state.load() == CamState::Acquiring)
            {
                SPDLOG_INFO("[OPTICAL] BeginAcquisition()");
                bool started_acq = false;

                try {
                    m_running.store(true);
                    m_cam->BeginAcquisition();
                    started_acq = true;
                } catch (const std::exception& e) {
                    SPDLOG_ERROR("[OPTICAL] BeginAcquisition failed: {}", e.what());
                    m_running.store(false);
                    m_state.store(CamState::Idle);
                    continue;
                }

                // 3) Acquisition loop (runs until stop acquisition or shutdown)
                int frame_count = 0;
                auto last_frame_time = std::chrono::steady_clock::now();

                int consec_fail = 0;        // exceptions / null frames / incomplete
                int consec_incomplete = 0;  // incomplete streak

                auto restart_acquisition = [&]() {
                    SPDLOG_WARN("[OPTICAL] Restarting acquisition (consec_fail={})", consec_fail);
                    try { if (started_acq) m_cam->EndAcquisition(); } catch (...) {}
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    try {
                        m_cam->BeginAcquisition();
                        started_acq = true;
                    } catch (const std::exception& e) {
                        started_acq = false;
                        SPDLOG_ERROR("[OPTICAL] Restart BeginAcquisition failed: {}", e.what());
                    }
                    consec_fail = 0;
                    consec_incomplete = 0;
                };

                while (!st.stop_requested()
                    && m_state.load() == CamState::Acquiring
                    && m_running.load())
                {
                    // Trigger mode: idle-wait but remain responsive
                    if (m_trigger_mode_enabled) {
                        std::unique_lock<std::mutex> lk(m_mtx);
                        m_conditionVariable.wait_for(lk, std::chrono::milliseconds(100), [&]{
                            return st.stop_requested()
                                || m_state.load() != CamState::Acquiring
                                || !m_running.load();
                        });
                        continue;
                    }

                    // ----- Throttle (same idea as your working run()) -----
                    auto now = std::chrono::steady_clock::now();
                    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_time).count();
                    if (elapsed_ms < m_throttle_ms) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(m_throttle_ms - elapsed_ms));
                    }

                    // ----- Grab with timeout -----
                    ImagePtr img;
                    try {
                        const unsigned timeout_ms = static_cast<unsigned>(
                            std::max<int>(static_cast<int>(m_throttle_ms) + 250, 1000)
                        );
                        img = m_cam->GetNextImage(timeout_ms);
                        // OG_CRITICAL("[OPTICAL] GetNextImage() returned after {} ms", timeout_ms);

                    } catch (const std::exception& e) {
                        SPDLOG_ERROR("[OPTICAL] GetNextImage failed: {}", e.what());
                        ++consec_fail;
                        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // backoff to avoid hammering
                        if (consec_fail >= 10) {
                            restart_acquisition();
                        }
                        continue;
                    }

                    if (!img) {
                        SPDLOG_WARN("[OPTICAL] GetNextImage returned null");
                        ++consec_fail;
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                        if (consec_fail >= 10) {
                            restart_acquisition();
                        }
                        continue;
                    }

                    if (img->IsIncomplete()) {
                        SPDLOG_WARN("[OPTICAL] Incomplete image; status={}",
                                    static_cast<int>(img->GetImageStatus()));
                        ++consec_fail;
                        ++consec_incomplete;
                        try { img->Release(); } catch (...) {}
                        if (consec_incomplete >= 10) {
                            restart_acquisition();
                        }
                        continue;
                    }

                    // Success path resets failure counters
                    consec_fail = 0;
                    consec_incomplete = 0;

                    try {
                        // Update last frame time ONLY after successful image retrieval
                        last_frame_time = std::chrono::steady_clock::now();

                        // ---- Use camera timestamp for BOTH RAW and JPEG filenames ----
                        const std::string stem = make_stem_from_image_timestamp(img, frame_count++);
                        const fs::path out_dir = fs::path(m_output_dir);

                        // ---- Save RAW (same as before, but uses stem) ----
                        {
                            const uint8_t* data = static_cast<const uint8_t*>(img->GetData());
                            const size_t size = img->GetBufferSize();

                            const fs::path raw_path = out_dir / (stem + ".raw");
                            std::ofstream raw_file(raw_path, std::ios::binary);
                            if (!raw_file.write(reinterpret_cast<const char*>(data), size)) {
                                SPDLOG_ERROR("[OPTICAL] Failed to save RAW: {}", raw_path.string());
                            } else {
                                SPDLOG_INFO("[OPTICAL] Saved RAW: {} ({} bytes)", raw_path.string(), size);
                            }
                        }

                        // ---- Save JPEG derived from the same camera frame ----
                        {
                            std::string jpeg_err;
                            const bool jpeg_ok = save_jpeg_from_image(
                                img,
                                out_dir,
                                stem,
                                85,        // quality
                                &jpeg_err
                            );

                            if (!jpeg_ok) {
                                SPDLOG_WARN("[OPTICAL] JPEG save failed for stem='{}': {}", stem, jpeg_err);
                            }
                        }

                        // IMPORTANT: always release
                        img->Release();

                    } catch (const std::exception& e) {
                        SPDLOG_ERROR("[OPTICAL] Error processing frame: {}", e.what());
                        try { img->Release(); } catch (...) {}
                    }


                    // Wakeable wait (lets stopAcquisition be responsive)
                    std::unique_lock<std::mutex> lk(m_mtx);
                    m_conditionVariable.wait_for(lk, std::chrono::milliseconds(m_throttle_ms), [&]{
                        return st.stop_requested()
                            || m_state.load() != CamState::Acquiring
                            || !m_running.load();
                    });
                }

                // 4) Stop acquisition cleanly
                SPDLOG_INFO("[OPTICAL] EndAcquisition()");
                try {
                    if (started_acq) {
                        m_cam->EndAcquisition();
                    }
                } catch (const std::exception& e) {
                    SPDLOG_WARN("[OPTICAL] EndAcquisition threw: {}", e.what());
                }

                m_running.store(false);

                // If we were acquiring and got told to stop acquiring, go Idle.
                if (m_state.load() != CamState::Shutdown && !st.stop_requested()) {
                    m_state.store(CamState::Idle);
                    SPDLOG_INFO("[OPTICAL] State=Idle (acquisition stopped)");
                }
            }
        }

        SPDLOG_INFO("[OPTICAL] Camera thread exiting");
    }


    bool FLIR_Blackfly_S::stop()
    {
        // Stop acquisition loop (if running) AND stop the thread
        m_running.store(false);
        m_state.store(CamState::Shutdown);

        // Wake the thread if it's waiting
        m_conditionVariable.notify_all();

        // Ask jthread to stop (sets stop_token)
        if (m_optical_thread.joinable())
        {
            m_optical_thread.request_stop();
            m_optical_thread.join();
        }

        // Cleanup after thread has exited (safe)
        try {
            if (m_cam)
            {
                // EndAcquisition only if grabbing; otherwise it throws.
                // If you have a way to query "is grabbing", use it.
                // Otherwise just try/catch.
                try { m_cam->EndAcquisition(); } catch (...) {}
                try { m_cam->DeInit(); } catch (...) {}
                m_cam = nullptr;
            }
            if (m_cam_list.GetSize() > 0) {
                try { m_cam_list.Clear(); } catch (...) {}
            }
            if (m_system) {
                try { m_system->ReleaseInstance(); } catch (...) {}
                m_system = nullptr;
            }
        } catch (...) {}

        return true;
    }

    bool FLIR_Blackfly_S::startAcquisition()
    {
        if (!m_optical_thread.joinable()) {
            SPDLOG_WARN("[OPTICAL] startAcquisition called but camera thread not running");
            return false;
        }

        {
            std::lock_guard<std::mutex> lk(m_mtx);

            const CamState current = m_state.load();
            SPDLOG_INFO("[OPTICAL] startAcquisition() called — current state={}",
                        to_string(current));

            if (current == CamState::Shutdown) {
                SPDLOG_WARN("[OPTICAL] Cannot start acquisition: state=Shutdown");
                return false;
            }

            if (current == CamState::Acquiring) {
                SPDLOG_INFO("[OPTICAL] Already acquiring — no state change");
                return true;
            }

            // Transition Idle -> Acquiring
            m_state.store(CamState::Acquiring);
            SPDLOG_INFO("[OPTICAL] State transition: {} -> Acquiring",
                        to_string(current));
        }

        m_conditionVariable.notify_all();
        return true;
    }

    bool FLIR_Blackfly_S::stopAcquisition()
    {
        if (!m_optical_thread.joinable()) return false;

        {
            std::lock_guard<std::mutex> lk(m_mtx);

            const auto s = m_state.load();
            if (s == CamState::Shutdown) return false;
            if (s == CamState::Idle) return true;        // already idle

            m_state.store(CamState::Idle);
            m_running.store(false); // helps the inner loop exit quickly
        }

        m_conditionVariable.notify_all();
        return true;
    }

}
