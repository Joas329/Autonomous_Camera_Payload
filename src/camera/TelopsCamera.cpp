// *****************************************************************************
//
//     Copyright (c) 2025, SOBER Team, All rights reserved.
//     Telops RadiaM100 camera driver
//     Author: Leroy Musa, Joaquin Philco
//
// *****************************************************************************

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <filesystem>
#include <condition_variable>
#include <cstdint>
#include <time.h>

// eBUS SDK includes (Pleora Technologies)
#include <PvDevice.h>
#include <PvStream.h>
#include <PvBuffer.h>
#include <PvSystem.h>
#include <PvResult.h>
#include <PvDeviceSerialPort.h>
#include <PvDeviceAdapter.h>
#include <PvBufferWriter.h>

// Telops SDK includes (Telops-specific functionality)
#include "Telops/SerialCommunicationManager.h"
#include "Telops/OperationalCommandRead.h"
#include "Telops/OperationalCommandWrite.h"
#include "camera/TelopsCamera.hpp"

namespace sober::camera {

TelopsTriggerSystem::TelopsTriggerSystem(sober::eventbus::EventBus& bus, std::string name)
    : m_bus(bus),
      m_name(std::move(name)),
      m_system(nullptr),
      m_device(nullptr),
      m_stream(nullptr),
      m_pvDeviceInfo(nullptr),
      m_serialManager(std::make_unique<Telops::SerialCommunicationManager>()),
      m_operationalCommandRead(std::make_unique<Telops::OperationalCommandRead>()),
      m_operationalCommandWrite(std::make_unique<Telops::OperationalCommandWrite>()),
      m_powerSupplies(std::make_unique<Telops::PowerSuppliesRead>()),
      m_lensTemperature(std::make_unique<Telops::ReadInternalLensTemperature>()),
      m_chunkHeader(),
      m_bufferList(),
      m_capturedImages(),
      m_initialized(false),
      m_captureMode(TelopsTriggerSystem::CaptureMode::SingleFrame),
      m_integrationTimeUs(20000),
      m_imageWidth(640),
      m_imageHeight(512),
      m_cachedFpaTemperature(-273.15f),
      m_cachedLensTemperature(-273.15f),
      m_cachedBoardTemperature(-273.15f),
      m_lastTemperatureRead(std::chrono::steady_clock::now()),
      m_temperatureCacheDuration(std::chrono::milliseconds(100)),
      m_defaultIntegrationTimeUs(20000),
      m_defaultFrametimeUs(33333),
      m_defaultImageWidth(640),
      m_defaultImageHeight(512),
      m_maxStreamBuffers(10),
      m_captureTimeoutMs(1000),
      m_tempThreadRunning(false),
      m_tempUpdateThread(),
      m_running(false),
      m_saveDir("ir") {

    //Create save directory
    std::filesystem::create_directories(m_saveDir);
}

TelopsTriggerSystem::~TelopsTriggerSystem() {
    stop();
}

// Thread control methods
void TelopsTriggerSystem::start() {
    if (m_running) {
        return;
    }

    // Initialize the camera first
    if (!Initialize()) {
        std::cerr << "Failed to initialize " << m_name << " camera!" << std::endl;
        return;
    }

    m_initialized = true;
    m_running = true;

    // Start the main capture thread
    // m_thread = std::thread(&TelopsTriggerSystem::run, this);

    // Start temperature monitoring thread
    StartTemperatureThread();

    std::cout << m_name << " camera started successfully" << std::endl;
}

void TelopsTriggerSystem::stop() {
    if (!m_running.exchange(false)) return;  // idempotent

    // Signal the thread to wake up
    m_condition.notify_all();

    // Wait for the main thread to finish
    if (m_thread.joinable()) {
        m_thread.join();
    }

    // Stop temperature thread
    StopTemperatureThread();

    // Cleanup eBUS SDK resources
    Cleanup();

    std::cout << m_name << " camera stopped" << std::endl;
}

void TelopsTriggerSystem::run() {
    std::cout << m_name << " camera thread started" << std::endl;

    while (m_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << m_name << " camera thread stopped" << std::endl;
}

bool TelopsTriggerSystem::IsInitialized() const {
    return m_initialized;
}

TelopsTriggerSystem::CaptureMode TelopsTriggerSystem::GetCaptureMode() const {
    return m_captureMode;
}

uint32_t TelopsTriggerSystem::GetImageWidth() const {
    return m_imageWidth;
}

uint32_t TelopsTriggerSystem::GetImageHeight() const {
    return m_imageHeight;
}

uint32_t TelopsTriggerSystem::GetDefaultIntegrationTimeUs() const {
    return m_defaultIntegrationTimeUs;
}

uint32_t TelopsTriggerSystem::GetDefaultFrametimeUs() const {
    return m_defaultFrametimeUs;
}

uint32_t TelopsTriggerSystem::GetMaxStreamBuffers() const {
    return m_maxStreamBuffers;
}

uint32_t TelopsTriggerSystem::GetCaptureTimeoutMs() const {
    return m_captureTimeoutMs;
}

bool TelopsTriggerSystem::Initialize() {
    try {
        // Step 1: Find Telops camera using eBUS SDK
        if (!FindTelopsCamera()) {
            return false;
        }

        // Step 2: Connect to device using eBUS SDK
        PvResult result;
        m_device = PvDevice::CreateAndConnect(m_pvDeviceInfo, &result);
        if (!m_device || !result.IsOK()) {
            return false;
        }

        // Step 3: Configure camera parameters using eBUS SDK
        if (!ConfigureCameraParameters()) {
            return false;
        }

        // Step 4: Configure software triggering using eBUS SDK
        if (!ConfigureForSoftwareTrigger()) {
            return false;
        }

        // Step 5: Create and open stream using eBUS SDK
        PvResult streamResult;
        m_stream = PvStream::CreateAndOpen(m_pvDeviceInfo->GetConnectionID(), &streamResult);
        if (!m_stream || !streamResult.IsOK()) {
            return false;
        }

        // Step 6: Create stream buffers using eBUS SDK
        if (!CreateStreamBuffers()) {
            return false;
        }

        // Step 6.5: Initialize the chunk header with proper values
        InitializeChunkHeader();

        // Step 7: Enable streaming using eBUS SDK
        PvResult enableResult = m_device->StreamEnable();
        if (!enableResult.IsOK()) {
            return false;
        }

        m_initialized = true; // Set initialized flag when initialization succeeds
        return true;

    } catch (const std::exception& e) {
        return false;
    }
}

void TelopsTriggerSystem::Cleanup() {
    StopTemperatureThread();
    if (!m_initialized) return;

    // Cleanup eBUS SDK components
    if (m_stream) {
        if (m_device) {
            PvGenParameterArray* params = m_device->GetParameters();
            if (params) {
                PvGenCommand* stopCmd = dynamic_cast<PvGenCommand*>(params->Get("AcquisitionStop"));
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

    //Now it is safe to delete PvBuffer objects
    CleanupBuffers();

    // Cleanup Telops SDK components
    if (m_serialManager) {
        m_serialManager->Close();
    }

    if (m_device) {
        m_device->Disconnect();
        PvDevice::Free(m_device);
        m_device = nullptr;
    }

    if (m_pvDeviceInfo) {
        delete m_pvDeviceInfo;
        m_pvDeviceInfo = nullptr;
    }

    if (m_system) {
        delete m_system;
        m_system = nullptr;
    }

    m_initialized = false;
}

bool TelopsTriggerSystem::FindTelopsCamera() {
    // Use eBUS SDK to find Telops camera
    PvResult result;
    m_system = new PvSystem();
    m_system->Find();

    if (m_system->GetInterfaceCount() == 0) {
        return false;
    }

    for (uint32_t i = 0; i < m_system->GetInterfaceCount(); ++i) {
        const PvInterface* iface = m_system->GetInterface(i);

        for (uint32_t j = 0; j < iface->GetDeviceCount(); ++j) {
            m_pvDeviceInfo = iface->GetDeviceInfo(j);
            std::string name = m_pvDeviceInfo->GetDisplayID().GetAscii();
            if (name.find(TELOPS_DEVICE_DISPLAY_ID) != std::string::npos) {
                return true;
            }
        }
    }

    return false;
}

bool TelopsTriggerSystem::ConfigureCameraParameters() {
    // Configure camera using eBUS SDK parameters
    PvGenParameterArray* params = m_device->GetParameters();
    if (!params) {
        return false;
    }

    PvResult result;

    // Set pixel format to Mono16
    PvGenEnum* pixelFormat = dynamic_cast<PvGenEnum*>(params->Get("PixelFormat"));
    if (pixelFormat) {
        result = pixelFormat->SetValue("Mono16");
    }

    // Set acquisition mode to continuous
    PvGenEnum* acquisitionMode = dynamic_cast<PvGenEnum*>(params->Get("AcquisitionMode"));
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

    if (width) width->SetValue(m_imageWidth);
    if (height) height->SetValue(m_imageHeight);
    if (offsetX) offsetX->SetValue(0);
    if (offsetY) offsetY->SetValue(0);

    return true;
}

bool TelopsTriggerSystem::ConfigureForSoftwareTrigger() {

    if (!m_serialManager->Open(m_device)) {
        return false;
    }

    // Configure the mux for lens temperature communication (as per Matthieu's example)
    PvResult muxResult = m_serialManager->ConfigureMux();
    if (!muxResult.IsOK()) {
        return false;
    }

    if (!m_serialManager->ProcessReadOperationalCommand(m_operationalCommandRead.get())) {
        return false;
    }

    *m_operationalCommandWrite = *m_operationalCommandRead;

        m_operationalCommandWrite->SetImageVerticalLength(256);
        m_operationalCommandWrite->SetImageHorizontalLength(160);
        m_operationalCommandWrite->SetImageVerticalOffset(0);
        m_operationalCommandWrite->SetImageHorizontalOffset(0);
        m_operationalCommandWrite->SetMasterSlaveSync(1);
        m_operationalCommandWrite->SetIntegrationTime(m_integrationTimeUs);
        m_operationalCommandWrite->SetFrametime(GetDefaultFrametimeUs());
        m_operationalCommandWrite->SetPixelGainMode(2);
        m_operationalCommandWrite->SetFrameReadDelay(16);
        m_operationalCommandWrite->SetIntegrationDelay(8);
        m_operationalCommandWrite->SetUpDown(0);
        m_operationalCommandWrite->SetLeftRight(1);
        m_operationalCommandWrite->SetHeaderDisable(1);
        m_operationalCommandWrite->SetDiodeBias(6);

    if (!m_serialManager->ProcessWriteOperationalCommand(m_operationalCommandWrite.get())) {
        return false;
    }

    return true;
}

bool TelopsTriggerSystem::CreateStreamBuffers() {
    const uint32_t payloadSize = m_device->GetPayloadSize();
    const uint32_t maxStreamBuffers = m_stream->GetQueuedBufferMaximum();
    const uint32_t count = std::min(maxStreamBuffers, GetMaxStreamBuffers());

    for (uint32_t i = 0; i < count; ++i) {
        PvBuffer* buf = new PvBuffer();
        buf->Alloc(payloadSize);
        m_bufferList.push_back(buf);
    }

    for (PvBuffer* buf : m_bufferList) {
        m_stream->QueueBuffer(buf);
    }

    return true;
}

void TelopsTriggerSystem::CleanupBuffers() {
    for (const auto& pair : m_capturedImages) {
        delete std::get<0>(pair);
    }
    m_capturedImages.clear();

    for (auto buffer : m_bufferList) {
        delete buffer;
    }
    m_bufferList.clear();
}

bool TelopsTriggerSystem::CaptureImage(const std::string& filename) {
    if (!m_initialized) {
        return false;
    }

    try {
        PvBuffer* received = CaptureImageBuffer();
        if (!received) {
            return false;
        }

        std::string timestampedFilename = GenerateTimestampedFilename(filename);
        // std::cout << "[Telops] Saving to: " << std::filesystem::absolute(timestampedFilename) << std::endl;
        bool success = SaveBufferAsRaw(received, timestampedFilename);
        return success;

    } catch (const std::exception& e) {
        return false;
    }
}

bool TelopsTriggerSystem::triggerFrameWithTimestamp(const std::string& filename, const std::string& sync_timestamp) {
    if (!m_initialized) {
        return false;
    }

    try {
        UpdateChunkHeader();
        PvBuffer* received = CaptureImageBuffer();
        if (!received) {
            return false;
        }

        // Use the synchronized timestamp from main.cpp
        std::stringstream ss;
        ss << m_saveDir << "/" << filename << "_" << sync_timestamp << ".hcc";
        std::string timestampedFilename = ss.str();

        // std::cout << "[Telops] Saving to: " << std::filesystem::absolute(timestampedFilename) << std::endl;

        bool success = SaveBufferAsTelopsRaw(received, timestampedFilename);
        return success;

    } catch (const std::exception& e) {
        return false;
    }
}

bool TelopsTriggerSystem::triggerFrame(const std::string& filename) {
    if (!m_initialized) {
        return false;
    }

    try {
        UpdateChunkHeader();
        PvBuffer* received = CaptureImageBuffer();
        if (!received) {
            return false;
        }

        std::string timestampedFilename = GenerateTimestampedFilename(filename);
        // std::cout << "[Telops] Saving to: " << std::filesystem::absolute(timestampedFilename) << std::endl;
        bool success = SaveBufferAsTelopsRaw(received, timestampedFilename);
        return success;

    } catch (const std::exception& e) {
        return false;
    }
}

bool TelopsTriggerSystem::CaptureImageWithHeader(const std::string& filename) {
    if (!m_initialized) {
        return false;
    }

    try {
        // Only read from thread
        //ReadTemperatures();
        UpdateChunkHeader();

        PvBuffer* received = CaptureImageBuffer();
        if (!received) {
            return false;
        }

        std::string timestampedFilename = GenerateTimestampedFilename(filename);
        // std::cout << "[Telops] Saving to: " << std::filesystem::absolute(timestampedFilename) << std::endl;
        bool success = SaveBufferAsTelopsRaw(received, timestampedFilename);
        return success;

    } catch (const std::exception& e) {
        return false;
    }
}

void TelopsTriggerSystem::SetCaptureMode(TelopsTriggerSystem::CaptureMode mode) {
    m_captureMode = mode;
    if (mode == TelopsTriggerSystem::CaptureMode::BatchMode) {
        m_capturedImages.clear();
    }
}

bool TelopsTriggerSystem::CaptureImageInMode() {
    if (!m_initialized) {
        return false;
    }

    try {
        PvBuffer* received = CaptureImageBuffer();
        if (!received) {
            return false;
        }

        if (m_captureMode == TelopsTriggerSystem::CaptureMode::BatchMode) {
            //Update header at capture time and store a copy
            UpdateChunkHeader();
            Telops::ChunkDataHeaderStruct headerCopy = m_chunkHeader;
            PvBuffer* bufferCopy = new PvBuffer();
            bufferCopy->Alloc(received->GetImage()->GetImageSize());
            memcpy(bufferCopy->GetImage()->GetDataPointer(),
                   received->GetImage()->GetDataPointer(),
                   received->GetImage()->GetImageSize());

            std::string filename = GenerateTimestampedFilename("telops_batch");
            m_capturedImages.push_back(std::make_tuple(bufferCopy, filename, headerCopy));
        }

        return true;

    } catch (const std::exception& e) {
        return false;
    }
}

bool TelopsTriggerSystem::SaveBatchImages() {
    bool allSuccess = true;

    for (const auto& tuple : m_capturedImages) {
        PvBuffer* buffer = std::get<0>(tuple);
        const std::string& filename = std::get<1>(tuple);
        const Telops::ChunkDataHeaderStruct& header = std::get<2>(tuple);
        if (!SaveBufferAsTelopsRaw(buffer, filename, &header)) {
            allSuccess = false;
        }
        delete buffer;
    }

    m_capturedImages.clear();
    return allSuccess;
}

bool TelopsTriggerSystem::SetIntegrationTime(uint64_t integrationTimeUs) {

    if (!m_operationalCommandWrite || !m_serialManager) {
        return false;
    }

    if (!m_serialManager->IsOpen()) {
        return false;
    }

    m_integrationTimeUs = integrationTimeUs;
        m_operationalCommandWrite->SetIntegrationTime(integrationTimeUs);

    if (!m_serialManager->ProcessWriteOperationalCommand(m_operationalCommandWrite.get())) {
        return false;
    }

    return true;
}

uint64_t TelopsTriggerSystem::GetIntegrationTime() const {
    return m_integrationTimeUs;
}

bool TelopsTriggerSystem::SetFrametime(uint64_t frametimeUs) {
    if (!m_operationalCommandWrite || !m_serialManager) {
        return false;
    }

    if (!m_serialManager->IsOpen()) {
        return false;
    }

    // Update the operational command with new frame time
    m_operationalCommandWrite->SetFrametime(frametimeUs);

    // Send the command to the camera
    if (!m_serialManager->ProcessWriteOperationalCommand(m_operationalCommandWrite.get())) {
        return false;
    }

    // Update our internal default frame time
    m_defaultFrametimeUs = frametimeUs;

    return true;
}

uint64_t TelopsTriggerSystem::GetFrametime() const {
    if (m_operationalCommandWrite) {
        return m_operationalCommandWrite->GetFrametime();
    }
    return m_defaultFrametimeUs;
}

bool TelopsTriggerSystem::ReadTemperatures() {
    if (!m_initialized || !m_serialManager || !m_serialManager->IsOpen()) {
        return false;
    }
    bool ok = true;
    // FPA and board temp
    if (!UpdatePowerSuppliesTemperatures()) {
        ok = false;
    }
    // Lens temp
    if (m_lensTemperature) {
        if (m_serialManager->ProcessReadLensTemperature(*m_lensTemperature)) {
            m_cachedLensTemperature = m_lensTemperature->GetInternalLensTemperature() / 100.0f;
        } else {
            m_cachedLensTemperature = -273.15f;
            ok = false;
        }
    } else {
        m_cachedLensTemperature = -273.15f;
        ok = false;
    }
    m_lastTemperatureRead = std::chrono::steady_clock::now();
    return ok;
}

float TelopsTriggerSystem::GetFpaTemperature() const {
    //std::lock_guard<std::mutex> lock(m_tempMutex);
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTemperatureRead) > m_temperatureCacheDuration) {
        if (m_initialized && m_serialManager && m_serialManager->IsOpen()) {
            const_cast<TelopsTriggerSystem*>(this)->UpdatePowerSuppliesTemperatures();
        }
    }
    return m_cachedFpaTemperature;
}

float TelopsTriggerSystem::GetLensTemperature() const {
    //std::lock_guard<std::mutex> lock(m_tempMutex);
    // Check if cache is still valid
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTemperatureRead) > m_temperatureCacheDuration) {
        // Cache expired, trigger a new read
        if (m_initialized && m_serialManager && m_serialManager->IsOpen()) {
            const_cast<TelopsTriggerSystem*>(this)->ReadTemperatures();
        }
    }
    return m_cachedLensTemperature;
}

float TelopsTriggerSystem::GetBoardTemperature() const {
   // std::lock_guard<std::mutex> lock(m_tempMutex);
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - m_lastTemperatureRead) > m_temperatureCacheDuration) {
        if (m_initialized && m_serialManager && m_serialManager->IsOpen()) {
            const_cast<TelopsTriggerSystem*>(this)->UpdatePowerSuppliesTemperatures();
        }
    }
    return m_cachedBoardTemperature;
}

Telops::PowerSuppliesRead TelopsTriggerSystem::GetPowerSupplies() const {
    // Since we can't read power supplies through serial commands, return default values
    // The temperature data is available in the image header instead
    Telops::PowerSuppliesRead powerSupplies;

    // Set default values
    powerSupplies.Reset();

    // We could potentially read these from the image header if needed
    // For now, return default values
    return powerSupplies;
}

void TelopsTriggerSystem::UpdateChunkHeader() {
    if (!m_initialized) {
        return;
    }

    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()) % 1000000;

    m_chunkHeader.ChunkFrameIDValue++;
    m_chunkHeader.ChunkPOSIXTimeValue = static_cast<uint32_t>(time_t);
    m_chunkHeader.ChunkSubSecondTimeValue = static_cast<uint32_t>(microseconds.count() * 10);

    //Use cached temperature values from the temperature thread
    m_chunkHeader.ChunkTemperatureSensorValue = static_cast<int16_t>(m_cachedFpaTemperature * 100.0f);
    m_chunkHeader.ChunkTemperatureInternalLensValue = static_cast<int16_t>(m_cachedLensTemperature * 100.0f);

    if (m_operationalCommandWrite) {
        uint64_t integrationTime = m_operationalCommandWrite->GetIntegrationTime();
        m_chunkHeader.ChunkExposureTimeValue = static_cast<uint32_t>(integrationTime * 100);
        uint64_t frameTime = m_operationalCommandWrite->GetFrametime();
        float frameRateHz = 1000000.0f / static_cast<float>(frameTime);
        m_chunkHeader.ChunkAcquisitionFrameRateValue = static_cast<uint32_t>(frameRateHz * 1000.0f);
    }
}

void TelopsTriggerSystem::InitializeChunkHeader() {
    // Initialize header with proper Telops values
    m_chunkHeader = Telops::ChunkDataHeaderStruct(); // Use default constructor

    // Explicitly zero out all spare fields to ensure clean header
    memset(&m_chunkHeader.Spare001, 0, sizeof(m_chunkHeader.Spare001));
    memset(&m_chunkHeader.Spare002, 0, sizeof(m_chunkHeader.Spare002));
    memset(&m_chunkHeader.Spare003, 0, sizeof(m_chunkHeader.Spare003));
    memset(&m_chunkHeader.Spare004, 0, sizeof(m_chunkHeader.Spare004));
    memset(&m_chunkHeader.Spare005, 0, sizeof(m_chunkHeader.Spare005));
    memset(&m_chunkHeader.Spare006, 0, sizeof(m_chunkHeader.Spare006));
    memset(&m_chunkHeader.Spare007, 0, sizeof(m_chunkHeader.Spare007));
    memset(&m_chunkHeader.Spare008, 0, sizeof(m_chunkHeader.Spare008));
    memset(&m_chunkHeader.Spare009, 0, sizeof(m_chunkHeader.Spare009));
    memset(&m_chunkHeader.Spare010, 0, sizeof(m_chunkHeader.Spare010));
    memset(&m_chunkHeader.Spare0011, 0, sizeof(m_chunkHeader.Spare0011));
    memset(&m_chunkHeader.Spare0012, 0, sizeof(m_chunkHeader.Spare0012));

    // Set integration time and frame rate
    m_chunkHeader.ChunkExposureTimeValue = static_cast<uint32_t>(m_integrationTimeUs * 100ULL);
    m_chunkHeader.ChunkAcquisitionFrameRateValue = static_cast<uint32_t>((1000000.0f / static_cast<float>(m_defaultFrametimeUs)) * 1000.0f);

    // Initialize frame ID
    m_chunkHeader.ChunkFrameIDValue = 0;

    //Note: Image dimensions and header length will be set in SaveBufferAsTelopsRaw()
    //based on actual image dimensions from the buffer
}

PvBuffer* TelopsTriggerSystem::CaptureImageBuffer() {
    if (!m_initialized) {
        return nullptr;
    }

    try {
        PvGenCommand* startCmd = dynamic_cast<PvGenCommand*>(m_device->GetParameters()->Get("AcquisitionStart"));
        if (!startCmd) {
            return nullptr;
        }

        startCmd->Execute();

        PvBuffer* received = nullptr;
        PvResult opResult, result;
        result = m_stream->RetrieveBuffer(&received, &opResult, GetCaptureTimeoutMs());

        if (result.IsOK() && opResult.IsOK() && received) {
            if (received->GetPayloadType() == PvPayloadTypeImage) {
                PvImage* image = received->GetImage();
                if (image && image->GetDataPointer() && image->GetImageSize() > 0) {
                    return received;
                }
            }
            m_stream->QueueBuffer(received);
        } else {
            if (received) {
                m_stream->QueueBuffer(received);
            }
        }

    } catch (const std::exception& e) {
    }

    return nullptr;
}

bool TelopsTriggerSystem::SaveBufferAsRaw(PvBuffer* buffer, const std::string& filename) {
    if (!buffer) {
        return false;
    }

    PvImage* image = buffer->GetImage();
    if (!image || !image->GetDataPointer()) {
        return false;
    }

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }

    auto t_write_start = std::chrono::steady_clock::now();
    file.write(reinterpret_cast<const char*>(image->GetDataPointer()), image->GetImageSize());
    auto t_write_end = std::chrono::steady_clock::now();
    auto d = t_write_end - t_write_start;
    std::cout << "[Telops] SD write time: " << static_cast<double>(
               std::chrono::duration_cast<std::chrono::microseconds>(d)
                   .count()) /
           1000.0 << " ms" << std::endl;

    bool success = file.good();
    file.close();

    return success;
}

bool TelopsTriggerSystem::SaveBufferAsTelopsRaw(PvBuffer* buffer, const std::string& filename, const Telops::ChunkDataHeaderStruct* headerOverride) {
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
    std::memcpy(out.data() + 2 * line_bytes, image->GetDataPointer(), static_cast<size_t>(width) * static_cast<size_t>(height) * 2);

    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) return false;

    auto t_write_start = std::chrono::steady_clock::now();
    file.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(frame_bytes));
    auto t_write_end = std::chrono::steady_clock::now();
    auto d = t_write_end - t_write_start;
    std::cout << "[Telops] SD write time: " << static_cast<double>(
               std::chrono::duration_cast<std::chrono::microseconds>(d)
                   .count()) /
           1000.0 << " ms" << std::endl;

    return file.good();
}

std::string TelopsTriggerSystem::GenerateTimestampedFilename(const std::string& baseName,
                                                            const std::string& extension) const {
    // Generate filename with UTC timestamp using clock_gettime (verbatim implementation)
    struct timespec unixTimestamp {};
    if (clock_gettime(CLOCK_REALTIME, &unixTimestamp) != 0) {
        unixTimestamp.tv_sec = 0;
        unixTimestamp.tv_nsec = 0;
    }

    // Convert to UTC time for filename
    auto time = static_cast<time_t>(unixTimestamp.tv_sec);
    auto nanoseconds = unixTimestamp.tv_nsec;

    std::stringstream ss;
    ss << m_saveDir << "/" << baseName << "_" << std::put_time(std::gmtime(&time), "%Y%m%d_%H%M%S");
    ss << "_" << std::setfill('0') << std::setw(9) << nanoseconds;
    ss << extension;

    return ss.str();
}

bool TelopsTriggerSystem::UpdatePowerSuppliesTemperatures() {
    if (!m_serialManager || !m_serialManager->IsOpen() || !m_powerSupplies) {
        return false;
    }
    if (!m_serialManager->ProcessReadPowerSupplies(*m_powerSupplies)) {
        return false;
    }
    m_cachedFpaTemperature = m_powerSupplies->GetFpaTemp(); // Already in Celsius based on polynomial calculation
    m_cachedBoardTemperature = m_powerSupplies->GetBoardTemp(); // Already in Celsius
    m_lastTemperatureRead = std::chrono::steady_clock::now();
    return true;
}

void TelopsTriggerSystem::StartTemperatureThread() {
    if (m_tempThreadRunning) return;
    m_tempThreadRunning = true;
    m_tempUpdateThread = std::thread(&TelopsTriggerSystem::TemperatureThreadFunc, this);
}

void TelopsTriggerSystem::StopTemperatureThread() {
    m_tempThreadRunning = false;
    if (m_tempUpdateThread.joinable()) {
        m_tempUpdateThread.join();
    }
}

void TelopsTriggerSystem::TemperatureThreadFunc() {
    while (m_tempThreadRunning) {
        {
            //std::lock_guard<std::mutex> lock(m_tempMutex);
            ReadTemperatures();
        }
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }
}

bool TelopsTriggerSystem::SetIntegrationTimeWithStreamRestart(uint64_t integrationTimeUs) {
    if (!m_initialized) {
        std::cerr << "[Telops] Cannot change exposure time - camera not initialized" << std::endl;
        return false;
    }

    std::cout << "[Telops] Changing exposure time to " << integrationTimeUs << " us with stream restart" << std::endl;

    // Store current parameters before closing stream
    uint64_t currentIntegrationTime = m_integrationTimeUs;
    uint32_t currentWidth = m_imageWidth;
    uint32_t currentHeight = m_imageHeight;
    uint64_t currentFrametime = m_defaultFrametimeUs;

    try {
        // Step 1: Stop streaming and close stream
        if (m_device) {
            PvGenParameterArray* params = m_device->GetParameters();
            if (params) {
                PvGenCommand* stopCmd = dynamic_cast<PvGenCommand*>(params->Get("AcquisitionStop"));
                if (stopCmd) {
                    stopCmd->Execute();
                }
            }
            m_device->StreamDisable();
        }

        if (m_stream) {
            m_stream->AbortQueuedBuffers();
            // Dequeue all buffers from the stream before freeing
            PvBuffer* buf = nullptr;
            PvResult opResult, result;
            while (m_stream->RetrieveBuffer(&buf, &opResult, 0).IsOK() && buf) {
                // Do not delete here, just dequeue
            }
            PvStream::Free(m_stream);
            m_stream = nullptr;
        }

        // Step 2: Clean up buffers
        CleanupBuffers();

        // Step 3: Update integration time
        m_integrationTimeUs = integrationTimeUs;

        // Step 4: Reconfigure operational commands with new exposure time
        if (m_operationalCommandWrite && m_serialManager && m_serialManager->IsOpen()) {
            m_operationalCommandWrite->SetIntegrationTime(integrationTimeUs);
            m_operationalCommandWrite->SetFrametime(currentFrametime);
            m_operationalCommandWrite->SetImageVerticalLength(currentHeight / 2);
            m_operationalCommandWrite->SetImageHorizontalLength(currentWidth / 4);
            m_operationalCommandWrite->SetImageVerticalOffset(0);
            m_operationalCommandWrite->SetImageHorizontalOffset(0);
            m_operationalCommandWrite->SetMasterSlaveSync(1);
            m_operationalCommandWrite->SetPixelGainMode(2);
            m_operationalCommandWrite->SetFrameReadDelay(16);
            m_operationalCommandWrite->SetIntegrationDelay(8);
            m_operationalCommandWrite->SetUpDown(0);
            m_operationalCommandWrite->SetLeftRight(1);
            m_operationalCommandWrite->SetHeaderDisable(1);
            m_operationalCommandWrite->SetDiodeBias(6);

            if (!m_serialManager->ProcessWriteOperationalCommand(m_operationalCommandWrite.get())) {
                std::cerr << "[Telops] Failed to write operational command after exposure change" << std::endl;
                // Restore previous values
                m_integrationTimeUs = currentIntegrationTime;
                return false;
            }
        }

        // Step 5: Reopen stream
        PvResult streamResult;
        m_stream = PvStream::CreateAndOpen(m_pvDeviceInfo->GetConnectionID(), &streamResult);
        if (!m_stream || !streamResult.IsOK()) {
            std::cerr << "[Telops] Failed to reopen stream after exposure change" << std::endl;
            // Restore previous values
            m_integrationTimeUs = currentIntegrationTime;
            return false;
        }

        // Step 6: Recreate stream buffers
        if (!CreateStreamBuffers()) {
            std::cerr << "[Telops] Failed to recreate stream buffers after exposure change" << std::endl;
            // Restore previous values
            m_integrationTimeUs = currentIntegrationTime;
            return false;
        }

        // Step 7: Re-enable streaming
        PvResult enableResult = m_device->StreamEnable();
        if (!enableResult.IsOK()) {
            std::cerr << "[Telops] Failed to re-enable streaming after exposure change" << std::endl;
            // Restore previous values
            m_integrationTimeUs = currentIntegrationTime;
            return false;
        }

        // Step 8: Update chunk header with new exposure time
        InitializeChunkHeader();

        std::cout << "[Telops] Successfully changed exposure time to " << integrationTimeUs << " us" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Telops] Exception during exposure time change: " << e.what() << std::endl;
        // Restore previous values
        m_integrationTimeUs = currentIntegrationTime;
        return false;
    }
}

bool TelopsTriggerSystem::SetIntegrationTimeExampleStyle(uint64_t integrationTimeUs) {
    if (!m_initialized || !m_operationalCommandWrite || !m_serialManager) {
        std::cerr << "[Telops] Cannot change exposure time - camera not initialized or missing components" << std::endl;
        return false;
    }

    if (!m_serialManager->IsOpen()) {
        std::cerr << "[Telops] Serial communication not open" << std::endl;
        return false;
    }

    std::cout << "[Telops] Changing exposure time to " << integrationTimeUs << " us (example style)" << std::endl;

    try {
        // Update the operational command with new integration time
        m_operationalCommandWrite->SetIntegrationTime(integrationTimeUs);

        // Write the operational command to the camera
        bool success = m_serialManager->ProcessWriteOperationalCommand(m_operationalCommandWrite.get());
        if (!success) {
            std::cerr << "[Telops] Failed to write operational command for exposure change" << std::endl;
            return false;
        }

        // Update our internal state
        m_integrationTimeUs = integrationTimeUs;

        // Update the chunk header with new exposure time
        if (m_operationalCommandWrite) {
            uint64_t integrationTime = m_operationalCommandWrite->GetIntegrationTime();
            m_chunkHeader.ChunkExposureTimeValue = static_cast<uint32_t>(integrationTime * 100);
        }

        std::cout << "[Telops] Successfully changed exposure time to " << integrationTimeUs << " us (example style)" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Telops] Exception during exposure time change (example style): " << e.what() << std::endl;
        return false;
    }
}

bool TelopsTriggerSystem::ResetToDefaultParameters() {
    if (!m_initialized) {
        std::cerr << "[Telops] Cannot reset parameters - camera not initialized" << std::endl;
        return false;
    }

    std::cout << "[Telops] Resetting to default parameters" << std::endl;

    try {
        // Reset to default values
        m_integrationTimeUs = m_defaultIntegrationTimeUs;
        m_imageWidth = m_defaultImageWidth;
        m_imageHeight = m_defaultImageHeight;

        // Update operational command with default values
        if (m_operationalCommandWrite && m_serialManager && m_serialManager->IsOpen()) {
            m_operationalCommandWrite->SetIntegrationTime(m_defaultIntegrationTimeUs);
            m_operationalCommandWrite->SetFrametime(m_defaultFrametimeUs);
            m_operationalCommandWrite->SetImageVerticalLength(m_defaultImageHeight / 2);
            m_operationalCommandWrite->SetImageHorizontalLength(m_defaultImageWidth / 4);
            m_operationalCommandWrite->SetImageVerticalOffset(0);
            m_operationalCommandWrite->SetImageHorizontalOffset(0);
            m_operationalCommandWrite->SetMasterSlaveSync(1);
            m_operationalCommandWrite->SetPixelGainMode(2);
            m_operationalCommandWrite->SetFrameReadDelay(16);
            m_operationalCommandWrite->SetIntegrationDelay(8);
            m_operationalCommandWrite->SetUpDown(0);
            m_operationalCommandWrite->SetLeftRight(1);
            m_operationalCommandWrite->SetHeaderDisable(1);
            m_operationalCommandWrite->SetDiodeBias(6);

            if (!m_serialManager->ProcessWriteOperationalCommand(m_operationalCommandWrite.get())) {
                std::cerr << "[Telops] Failed to write default operational command" << std::endl;
                return false;
            }
        }

        // Update chunk header with default values
        InitializeChunkHeader();

        std::cout << "[Telops] Successfully reset to default parameters" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Telops] Exception during parameter reset: " << e.what() << std::endl;
        return false;
    }
}

void TelopsTriggerSystem::setSaveDir(const std::string& saveDir) {
    m_saveDir = saveDir;
    std::filesystem::create_directories(m_saveDir);
}

} // namespace sober::camera
