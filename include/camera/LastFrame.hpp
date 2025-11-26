#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <vector>

// extern std::atomic<std::shared_ptr<std::vector<uint8_t>>> g_lastFrame;
inline std::array<std::vector<uint8_t>, 2> buffers{};
inline std::atomic<size_t> current_idx{0};
inline std::atomic<uint32_t> originalSize{0};