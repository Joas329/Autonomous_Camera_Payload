// *****************************************************************************
//
//     Copyright (c) 2025, York Space Systems, All rights reserved.
//     FLIR Blackfly S BFS-U3-122S6C-C RGB camera driver
//     Author: Joaquin Philco
//
// *****************************************************************************

#include <cstddef>
#include <mutex>
#include <stop_token>
#include <thread>
#include <vector>

#include "camera/Ticker.hpp"

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <chrono>

#ifndef FLIR_BLACKFLY_S_ID
#define FLIR_BLACKFLY_S_ID "Blackfly S BFS-U3-122S6C"
#endif

namespace sober::camera {
    // Trigger source types supported by the camera
    enum class TriggerSource {
        Software,    // Software trigger
        Line0,       // Hardware trigger on Line 0
        Line1,       // Hardware trigger on Line 1
        Line2,       // Hardware trigger on Line 2
        Line3        // Hardware trigger on Line 3
    };

    // Trigger activation modes
    enum class TriggerActivation {
        RisingEdge,  // Trigger on rising edge
        FallingEdge, // Trigger on falling edge
        LevelHigh,   // Trigger on high level
        LevelLow     // Trigger on low level
    };

    class FLIR_Blackfly_S {
        public:
            FLIR_Blackfly_S(Ticker& ticker);
            ~FLIR_Blackfly_S();

            bool start();
            bool stopAcquisition();
            bool startAcquisition();

            bool stop();

            // // Frame rate control
            // void setFrameRate(double fps);
            // double getFrameRate() const;
            // double getMinFrameRate() const;
            // double getMaxFrameRate() const;

            // // Exposure control
            // float getExposureTime() const;
            // float getMinExposureTime() const;
            // float getMaxExposureTime() const;

        private:
            // ---- Camera thread state machine ----
            enum class CamState {
                Idle,        // configured, not acquiring
                Acquiring,   // actively grabbing frames
                Shutdown     // thread should exit
            };

            std::atomic<CamState> m_state{CamState::Idle};

            static const char* to_string(CamState s) {
                switch (s) {
                    case CamState::Idle:      return "Idle";
                    case CamState::Acquiring: return "Acquiring";
                    case CamState::Shutdown:  return "Shutdown";
                }
                return "Unknown";
            }

            // ---- Camera operation methods ---
            void main_camera_thread(std::stop_token stoken);
            void setup_camera();
            bool findFLIRCamera();
            void configureTriggerMode(bool enable);

            // Output Directory
            std::string m_output_dir;

            // Spinnaker handles
            Spinnaker::SystemPtr m_system;
            Spinnaker::CameraPtr m_cam;
            Spinnaker::CameraList m_cam_list;

            std::jthread m_optical_thread;
            std::atomic<bool> m_running;

            std::mutex m_mtx;
            std::condition_variable m_conditionVariable;

            // Default exposure time in microseconds
            double  m_default_exposure_us;

            // Exposure control
            float m_current_exposure_us;
            float m_min_exposure_us;
            float m_max_exposure_us;

            // Frame rate control
            double m_current_fps;
            double m_min_fps;
            double m_max_fps;

            // Trigger control
            bool m_trigger_mode_enabled;
            TriggerSource m_current_trigger_source;
            TriggerActivation current_trigger_activation;
            float m_trigger_delay_us;
            bool m_trigger_overlap;

            // Experimeetn Ticker
            sober::camera::Ticker& m_ticker;

            int m_timeout_ms;
            int m_target_fps;
            int m_throttle_ms;
    };

} // namespace sober::camera