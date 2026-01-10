#pragma once
// *****************************************************************************
//
//     Copyright (c) 2026, York Space Systems, All rights reserved.
//     FLIR Blackfly S BFS-U3-122S6C-C RGB camera driver
//     Author: Joaquin Philco
//
// *****************************************************************************

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <chrono>
#include <cstddef>
#include <deque>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "camera/Ticker.hpp"

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

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
        bool stop();

        bool startAcquisition();
        bool stopAcquisition();

        // Setters (Route A: queue commands; camera thread applies them in Idle)
        bool set_exposure_time(double exposure_us);
        bool set_frame_rate(double fps);
        bool set_output_dir(const std::string& dir);
        bool set_trigger_mode(bool enable);

    private:
        // ============================
        // Camera thread state machine
        // ============================
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

        // ============================
        // Command queue (Route A)
        // ============================
        enum class CmdType {
            SetExposure,
            SetFrameRate,
            SetOutputDir,
            SetTriggerMode
        };

        struct CmdSetExposure { double exposure_us; };
        struct CmdSetFrameRate { double fps; };
        struct CmdSetOutputDir { std::string dir; };
        struct CmdSetTriggerMode { bool enable; };

        using CmdPayload = std::variant<
            CmdSetExposure,
            CmdSetFrameRate,
            CmdSetOutputDir,
            CmdSetTriggerMode
        >;

        struct Cmd {
            CmdType type;
            CmdPayload payload;
        };

        // camera-thread only: apply queued commands when safe (typically in Idle)
        void apply_pending_commands();

        // camera-thread only: Spinnaker node writes
        bool apply_exposure_camera_thread(double exposure_us, std::string* err);
        bool apply_fps_camera_thread(double fps, std::string* err);
        bool apply_output_dir_camera_thread(const std::string& dir, std::string* err);
        bool apply_trigger_camera_thread(bool enable, std::string* err);

        // Queue storage
        std::mutex      m_cmd_mtx;
        std::deque<Cmd> m_cmd_q;

        // ============================
        // Camera operation methods
        // ============================
        void main_camera_thread(std::stop_token stoken);
        void setup_camera();
        bool findFLIRCamera();
        void configureTriggerMode(bool enable);

        // ============================
        // Output Directory
        // ============================
        std::string m_output_dir;

        // ============================
        // Spinnaker handles
        // ============================
        Spinnaker::SystemPtr  m_system;
        Spinnaker::CameraPtr  m_cam;
        Spinnaker::CameraList m_cam_list;

        // ============================
        // Threading primitives
        // ============================
        std::jthread              m_optical_thread;
        std::atomic<bool>         m_running{false};

        std::mutex                m_mtx;
        std::condition_variable   m_conditionVariable;

        // ============================
        // Default exposure time in microseconds
        // ============================
        double  m_default_exposure_us{100000.0};

        // Exposure control
        double m_current_exposure_us{0.0};
        double m_min_exposure_us{1.0};
        double m_max_exposure_us{1000000.0};

        // Frame rate control
        double m_current_fps{0.0};
        double m_min_fps{0.0};
        double m_max_fps{0.0};

        // Trigger control
        bool m_trigger_mode_enabled{false};
        TriggerSource m_current_trigger_source{TriggerSource::Software};
        TriggerActivation m_current_trigger_activation{TriggerActivation::RisingEdge};
        double m_trigger_delay_us{0.0};
        bool m_trigger_overlap{false};

        // Experiment ticker
        sober::camera::Ticker& m_ticker;

        // Capture timing
        int m_timeout_ms{1000};
        int m_target_fps{9};
        int m_throttle_ms{111}; // ~9 fps
    };

} // namespace sober::camera
