#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace transport {

// A vector whose entire contents can be logically cleared in O(1). Every slot carries a stamp and is
// considered "current" only when its stamp matches the active generation, so bumping the generation in
// reset() invalidates every slot at once without touching memory. Reads of a non-current slot return the
// configured default. Intended as reusable scratch for graph searches that run many times over the same
// vertex set (Dijkstra, A*, witness search, bidirectional CH query) to avoid reallocating and refilling a
// fresh vector on every query. To keep several values per slot, use a struct element type rather than
// several StampedVectors, so one stamp array (and one reset) covers them all.
template <typename T>
class StampedVector {
public:
    StampedVector(size_t size, T default_value)
        : default_(default_value), values_(size, default_value), stamps_(size, 0) {}

    // Invalidates every slot. Until a slot is set again, get() returns the default for it.
    void reset() {
        ++stamp_;
        if (stamp_ == 0) { // generation wrapped around: the one case that needs a real clear
            std::fill(stamps_.begin(), stamps_.end(), 0);
            stamp_ = 1;
        }
    }

    [[nodiscard]] T get(size_t index) const { return stamps_[index] == stamp_ ? values_[index] : default_; }

    void set(size_t index, T value) {
        stamps_[index] = stamp_;
        values_[index] = value;
    }

private:
    T default_;
    std::vector<T> values_;
    std::vector<uint32_t> stamps_;
    uint32_t stamp_ = 0;
};

} // namespace transport
