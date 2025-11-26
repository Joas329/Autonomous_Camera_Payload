#pragma once

#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <stop_token>
#include <thread>

#include <PvStream.h>
#include <PvSystem.h>
#include <Telops/OperationalCommandRead.h>
#include <Telops/OperationalCommandWrite.h>

#include "camera/Ticker.hpp"

namespace Telops {

#pragma pack(push, 1)
struct ChunkDataHeaderStruct {
  char ChunkSignatureValue[2] = {'T', 'C'};     // Do not modify
  uint8_t ChunkDeviceXMLMinorVersionValue = 1;  // Do not modify
  uint8_t ChunkDeviceXMLMajorVersionValue = 0;  // Do not modify
  uint16_t ChunkImageHeaderLengthValue =
      640 * 2 *
      2;  // width * 2 * pixel_size_in_bytes, Modify width if you change it
  uint16_t ChunkDeviceModelNumberValue = 0x8010;  // Do not modify
  uint32_t ChunkFrameIDValue = 0;     // Update with your frame ID at each frame
  float ChunkDataOffsetValue = 0.0f;  // Do not modify
  int8_t ChunkDataExpValue = 0;       // Do not modify
  char Spare001[1];                   // Do not modify
  uint16_t ChunkDeviceModelNumberCheckValue = 0x8010;  // Do not modify
  char Spare002[4];                                    // Do not modify
  uint32_t ChunkExposureTimeValue =
      1000 * 100;  // IntegrationTime(us) * 100, e.g. 1000us = 100000
  uint8_t ChunkCalibrationModeValue = 255;  // Do not modify
  uint8_t ChunkBPRAppliedValue = 0;         // Do not modify
  char Spare003[2];                         // Do not modify
  uint16_t ChunkWidthValue = 640;           // replace with your width
  uint16_t ChunkHeightValue = 512;          // replace with your height
  uint16_t ChunkOffsetXValue = 0;           // should stay 0
  uint16_t ChunkOffsetYValue = 0;           // should stay 0
  uint8_t ChunkReverseXValue = 0;           // Do not modify
  uint8_t ChunkReverseYValue = 0;           // Do not modify
  uint8_t ChunkTestImageSelectorValue = 0;  // Do not modify
  uint8_t ChunkSensorWellDepthValue = 0;    // Do not modify
  uint32_t ChunkAcquisitionFrameRateValue =
      30.0f *
      1000.0f;  // Modify with your acquisition frame rate (from SetFrametime), framerate should be in mHz, // e.g. 30Hz = 30000, 60Hz = 60000, etc.
  float ChunkTriggerDelayValue = 0;          // Do not modify
  uint8_t ChunkTriggerModeValue = 0;         // Do not modify
  uint8_t ChunkTriggerSourceValue = 0;       // Do not modify
  uint8_t ChunkIntegrationModeValue = 0;     // Do not modify
  char Spare004[4];                          // Do not modify
  uint8_t ChunkExposureAutoValue = 0;        // Do not modify
  float ChunkAECResponseTimeValue = 0;       // Do not modify
  float ChunkAECImageFractionValue = 0;      // Do not modify
  float ChunkAECTargetWellFillingValue = 0;  // Do not modify
  char Spare005[3];                          // Do not modify
  uint8_t ChunkFWModeValue = 0;              // Do not modify
  char Spare006[24];                         // Do not modify
  uint32_t ChunkPOSIXTimeValue = 0;  // Modify with your value, in seconds
  uint32_t ChunkSubSecondTimeValue =
      0 *
      10;  // Modify with your value, your value should be in us * 10, // e.g. 1000us = 10000, 500us = 5000, etc.
  uint8_t ChunkTimeSourceValue = 0;                         // Do not modify
  char Spare007[19];                                        // Do not modify
  uint8_t ChunkFWPositionValue = 9;                         // Do not modify
  uint8_t ChunkICUPositionValue = 3;                        // Do not modify
  uint8_t ChunkNDFilterPositionValue = 4;                   // Do not modify
  uint8_t ChunkEHDRIExposureIndexValue = 5;                 // Do not modify
  char Spare008[1];                                         // Do not modify
  uint8_t ChunkPostProcessedValue = 0;                      // Do not modify
  uint16_t ChunkSensorTemperatureRawValue = 0;              // Do not modify
  char Spare009[12];                                        // Do not modify
  float ChunkLowCutValue;                                   // Do not modify
  float ChunkHighCutValue;                                  // Do not modify
  float ChunkExternalBlackBodyTemperatureValue = -273.15f;  // Do not modify
  int16_t ChunkTemperatureSensorValue =
      0;  // static_cast<uint16_t>((powerSuppliesRead.GetFpaTemp() - 273.15) * 100.0);, do it before your acquisition in your case
  uint8_t ChunkSensorIDMSBValue = 0;  // Do not modify
  char Spare010[43];                  // Do not modify
  int16_t ChunkTemperatureInternalLensValue =
      0;  // serialCommunicationManager->ReadLensTemperature(m_readInternalLensTemperature), then = readInternalLensTemperature.GetInternalLensTemperature(), do it before your acquisition in your case
  uint32_t ChunkFlashSettingsPOSIXTimeValue = 0;     // Do not modify
  uint32_t ChunkCalibrationBlockPOSIXTimeValue = 0;  // Do not modify
  uint32_t ChunkExternalLensSerialNumberValue =
      241126002;  // Manufuacturer serial number of your lens for now, we'll provide the Telops # soon
  uint32_t ChunkManualFilterSerialNumberValue = 0;              // Do not modify
  uint8_t ChunkSensorIDValue = 0;                               // Do not modify
  uint8_t ChunkPixelDataResolutionValue = 16;                   // Do not modify
  char Spare0011[6];                                            // Do not modify
  uint8_t ChunkDeviceFirmwareMajorVersionValue = 0;             // Do not modify
  uint8_t ChunkDeviceFirmwareMinorVersionValue = 0;             // Do not modify
  uint8_t ChunkDeviceFirmwareBuildVersionValue = 0;             // Do not modify
  uint8_t ChunkDeviceCalibrationFilesMajorVersionValue = 2;     // Do not modify
  uint8_t ChunkDeviceCalibrationFilesMinorVersionValue = 7;     // Do not modify
  uint8_t ChunkDeviceCalibrationFilesSubMinorVersionValue = 0;  // Do not modify
  uint8_t ChunkFirmwareVersionYearValue = 0;                    // Do not modify
  uint8_t ChunkFirmwareVersionMonthValue = 0;                   // Do not modify
  uint8_t ChunkFirmwareVersionDayValue = 0;                     // Do not modify
  uint8_t ChunkFirmwareVersionHourValue = 0;                    // Do not modify
  char Spare0012[1];                                            // Do not modify
  uint8_t ChunkConnectTLMajorVersionValue = 0;                  // Do not modify
  uint8_t ChunkConnectTLMinorVersionValue = 0;                  // Do not modify
  uint8_t ChunkConnectTLSubMinorVersionValue = 0;               // Do not modify
  uint8_t ChunkDeviceDataFlowMajorVersionValue = 1;             // Do not modify
  uint8_t ChunkDeviceDataFlowMinorVersionValue = 1;             // Do not modify
  uint32_t ChunkDeviceSerialNumberValue = 12374;                // Do not modify
  uint32_t ChunkCalibrationCollectionPOSIXTimeValue = 0;        // Do not modify
};
#pragma pack(pop)

}  // namespace Telops

namespace sober::camera {

class IRCamera {
 public:
  explicit IRCamera(Ticker& tick) : ticker_(tick) {}

  bool start();
  void stop();

 private:
  void run(std::stop_token stoken);
  void run_test(std::stop_token stoken);
  std::jthread irCameraThread_;
  void cleanupEbus();
  PvBuffer* CaptureImageBuffer();
  void UpdateChunkHeader();
  bool SaveBufferAsTelopsRaw(
      PvBuffer* buffer, const std::string& filename,
      const Telops::ChunkDataHeaderStruct* headerOverride = nullptr);

  sober::camera::Ticker& ticker_;

  std::mutex cvMutex_;
  std::condition_variable cv_;

  PvSystem* m_system;
  PvDevice* m_device;
  PvStream* m_stream;
  const PvDeviceInfo* m_pvDeviceInfo;

  std::vector<PvBuffer*> m_bufferList;
  const uint32_t m_imageWidth{640};
  const uint32_t m_imageHeight{512};
  const uint32_t m_maxStreamBuffers{10};

  Telops::ChunkDataHeaderStruct m_chunkHeader;
  mutable float m_cachedFpaTemperature{-273.15F};
  mutable float m_cachedLensTemperature{-273.15F};
  mutable float m_cachedBoardTemperature{-273.15F};
  std::unique_ptr<Telops::OperationalCommandWrite> m_operationalCommandWrite;

  std::filesystem::path m_outputDir{
      "/home/raduo/Desktop/SOBER/sober-on-board-module/build"};
};

}  // namespace sober::camera