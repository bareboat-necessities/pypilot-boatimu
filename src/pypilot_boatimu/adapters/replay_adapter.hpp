#pragma once

#include <stddef.h>

namespace pypilot_boatimu {

template<typename Sample>
class ReplayAdapter {
public:
    ReplayAdapter() : samples_(0), count_(0), index_(0) {}
    ReplayAdapter(const Sample* samples, size_t count) : samples_(samples), count_(count), index_(0) {}

    void reset(const Sample* samples, size_t count) {
        samples_ = samples;
        count_ = count;
        index_ = 0;
    }

    void rewind() { index_ = 0; }
    size_t index() const { return index_; }
    size_t count() const { return count_; }
    bool has_next() const { return samples_ && index_ < count_; }

    const Sample* peek() const {
        if (!has_next()) {
            return 0;
        }
        return &samples_[index_];
    }

    bool next(Sample& out) {
        if (!has_next()) {
            return false;
        }
        out = samples_[index_++];
        return true;
    }

private:
    const Sample* samples_;
    size_t count_;
    size_t index_;
};

} // namespace pypilot_boatimu
