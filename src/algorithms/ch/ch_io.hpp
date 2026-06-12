#pragma once

#include "algorithms/ch/contraction_hierarchy.hpp"

#include <string>

namespace transport::ch {

// Magic bytes for CH binary files
constexpr uint32_t kChMagic = 0x54524348; // "TRCH"
constexpr uint32_t kChVersion = 1;

// Save/load the contraction hierarchy (rank array + both upward CSRs).
// Returns false on failure; throws std::runtime_error on malformed input.
[[nodiscard]] bool save_ch(const ContractionHierarchy &ch, const std::string &path);
[[nodiscard]] ContractionHierarchy load_ch(const std::string &path);

} // namespace transport::ch
