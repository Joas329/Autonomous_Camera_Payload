// *****************************************************************************
//
//     Copyright (c) 2025, SOBER Team, All rights reserved.
//     Authors: Leroy Musa, Joaquin Philco
//
// *****************************************************************************

#if !defined(SOBER_TELOPS_CAMERA_HPP)
#define SOBER_TELOPS_CAMERA_HPP

#include "eventbus/EventBus.hpp"
#include "eventbus/Event.hpp"

#include <PvDevice.h>
#include <PvStream.h>
#include <PvBuffer.h>
#include <PvSystem.h>
#include <PvResult.h>
#include <PvDeviceSerialPort.h>
#include <PvDeviceAdapter.h>
#include <PvBufferWriter.h>

#include <Telops/SerialCommunicationManager.h>
#include <Telops/OperationalCommandRead.h>
#include <Telops/OperationalCommandWrite.h>
#include <Telops/PowerSuppliesRead.h>
#include <Telops/ReadInternalLensTemperature.h>
#include <Telops/ChunkDataHeaderStruct.h>

#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

#ifndef TELOPS_DEVICE_DISPLAY_ID
#define TELOPS_DEVICE_DISPLAY_ID "iPORT-CL-U3-PT03-CL0UP02-128xU"
#endif

namespace sober::camera {

class TelopsTriggerSystem {
public:
    enum class CaptureMode {
        SingleFrame,
        BatchMode
    };

    enum class ImageFormat {
        Raw,
        TelopsRaw
    };

    TelopsTriggerSystem(sober::eventbus::EventBus& bus, std::string name = "TelopsIR");
    TelopsTriggerSystem(const TelopsTriggerSystem&) = delete;
    TelopsTriggerSystem& operator=(const TelopsTriggerSystem&) = delete;
    ~TelopsTriggerSystem();

    // Thread control methods following the same pattern as other components
    void start();
    void stop();
    bool is_running() const { return m_running; }

    // Initialization and cleanup
    bool Initialize();
    void Cleanup();
    bool IsInitialized() const;

    // Capture methods
    bool CaptureImage(const std::string& filename);
    bool CaptureImageWithHeader(const std::string& filename);
    bool triggerFrame(const std::string& filename);
    bool triggerFrameWithTimestamp(const std::string& filename, const std::string& sync_timestamp);
    void SetCaptureMode(CaptureMode mode);
    bool CaptureImageInMode();
    bool SaveBatchImages();

    // Configuration methods
    bool SetIntegrationTime(uint64_t integrationTimeUs);
    uint64_t GetIntegrationTime() const;

    // New exposure time management functions
    bool SetIntegrationTimeWithStreamRestart(uint64_t integrationTimeUs);
    bool SetIntegrationTimeExampleStyle(uint64_t integrationTimeUs);
    bool ResetToDefaultParameters();

    // Frame rate configuration
    bool SetFrametime(uint64_t frametimeUs);
    uint64_t GetFrametime() const;

    bool ConfigureForSoftwareTrigger();

    // Temperature monitoring
    bool ReadTemperatures();
    float GetFpaTemperature() const;
    float GetLensTemperature() const;
    float GetBoardTemperature() const;
    Telops::PowerSuppliesRead GetPowerSupplies() const;
    bool ReadTemperaturesFromSdk();

    // File management
    std::string GenerateTimestampedFilename(const std::string& baseName,
                                           const std::string& extension = ".hcc") const;

    // Getters
    CaptureMode GetCaptureMode() const;
    uint32_t GetImageWidth() const;
    uint32_t GetImageHeight() const;
    uint32_t GetDefaultIntegrationTimeUs() const;
    uint32_t GetDefaultFrametimeUs() const;
    uint32_t GetMaxStreamBuffers() const;
    uint32_t GetCaptureTimeoutMs() const;

    // Returns the directory where images are saved
    const std::string& getSaveDir() const { return m_saveDir; }

    // Sets the directory where images are saved
    void setSaveDir(const std::string& saveDir);

private:
    void run(); // Main thread function
    bool FindTelopsCamera();
    bool ConfigureCameraParameters();
    bool CreateStreamBuffers();
    void UpdateChunkHeader();
    void InitializeChunkHeader();
    PvBuffer* CaptureImageBuffer();
    bool SaveBufferAsRaw(PvBuffer* buffer, const std::string& filename);
    bool SaveBufferAsTelopsRaw(PvBuffer* buffer, const std::string& filename, const Telops::ChunkDataHeaderStruct* headerOverride = nullptr);
    void CleanupBuffers();
    bool UpdatePowerSuppliesTemperatures();

    // EventBus and component identification
    sober::eventbus::EventBus& m_bus;
    std::string m_name;

    // eBUS SDK components
    PvSystem* m_system;
    PvDevice* m_device;
    PvStream* m_stream;
    const PvDeviceInfo* m_pvDeviceInfo;
    std::unique_ptr<Telops::SerialCommunicationManager> m_serialManager;
    std::unique_ptr<Telops::OperationalCommandRead> m_operationalCommandRead;
    std::unique_ptr<Telops::OperationalCommandWrite> m_operationalCommandWrite;
    std::unique_ptr<Telops::PowerSuppliesRead> m_powerSupplies;
    std::unique_ptr<Telops::ReadInternalLensTemperature> m_lensTemperature;

    // Chunk header for Telops RAW format
    Telops::ChunkDataHeaderStruct m_chunkHeader;

    // Buffer management
    std::vector<PvBuffer*> m_bufferList;
    std::vector<std::tuple<PvBuffer*, std::string, Telops::ChunkDataHeaderStruct>> m_capturedImages;

    // State management
    bool m_initialized;
    CaptureMode m_captureMode;
    uint64_t m_integrationTimeUs;
    uint32_t m_imageWidth;
    uint32_t m_imageHeight;

    // Temperature caching
    mutable float m_cachedFpaTemperature;
    mutable float m_cachedLensTemperature;
    mutable float m_cachedBoardTemperature;
    mutable std::chrono::steady_clock::time_point m_lastTemperatureRead;
    std::chrono::milliseconds m_temperatureCacheDuration;

    // Default parameters
    uint32_t m_defaultIntegrationTimeUs;
    uint32_t m_defaultFrametimeUs;
    uint32_t m_defaultImageWidth;
    uint32_t m_defaultImageHeight;
    uint32_t m_maxStreamBuffers;
    uint32_t m_captureTimeoutMs;

    // Temperature monitoring thread
    std::atomic<bool> m_tempThreadRunning{false};
    std::thread m_tempUpdateThread;
    void TemperatureThreadFunc();
    void StartTemperatureThread();
    void StopTemperatureThread();

    // Threading state
    std::atomic<bool> m_running{false};
    std::mutex m_mutex;
    std::condition_variable m_condition;
    std::thread m_thread;

    // Save directory
    std::string m_saveDir;
};

} // namespace sober::camera

#endif // SOBER_TELOPS_CAMERA_HPP