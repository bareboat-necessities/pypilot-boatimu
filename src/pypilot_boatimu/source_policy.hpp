#pragma once

#include <stdint.h>
#include <string.h>

namespace pypilot_boatimu {

static const uint64_t default_boatimu_source_timeout_us = 8000000ULL;

enum class BoatImuSource : uint8_t {
    None = 0,
    TrustedFused = 1,
    OceanImu = 2,
    RTIMULib2 = 3,
    ExternalAhrs = 4,
    CalibratedImu = 5,
    HeadingOnly = 6,
    Replay = 7
};

inline uint8_t boatimu_source_priority(BoatImuSource source) {
    switch (source) {
    case BoatImuSource::TrustedFused:  return 1;
    case BoatImuSource::OceanImu:      return 2;
    case BoatImuSource::RTIMULib2:     return 3;
    case BoatImuSource::ExternalAhrs:  return 4;
    case BoatImuSource::CalibratedImu: return 5;
    case BoatImuSource::HeadingOnly:   return 6;
    case BoatImuSource::Replay:        return 7;
    case BoatImuSource::None:
    default:                           return 255;
    }
}

inline const char* boatimu_source_name(BoatImuSource source) {
    switch (source) {
    case BoatImuSource::TrustedFused:  return "trusted_fused";
    case BoatImuSource::OceanImu:      return "ocean_imu";
    case BoatImuSource::RTIMULib2:     return "rtimulib2";
    case BoatImuSource::ExternalAhrs:  return "external_ahrs";
    case BoatImuSource::CalibratedImu: return "calibrated_imu";
    case BoatImuSource::HeadingOnly:   return "heading_only";
    case BoatImuSource::Replay:        return "replay";
    case BoatImuSource::None:
    default:                           return "none";
    }
}

inline bool boatimu_device_id_equal(const char* a, const char* b) {
    if (a == 0) {
        a = "";
    }
    if (b == 0) {
        b = "";
    }
    return strcmp(a, b) == 0;
}

struct BoatImuSourceTag {
    BoatImuSource source;
    const char* device_id;

    BoatImuSourceTag() : source(BoatImuSource::None), device_id("") {}
    BoatImuSourceTag(BoatImuSource source_, const char* device_id_)
        : source(source_), device_id(device_id_ ? device_id_ : "") {}
};

class BoatImuSourceArbitrator {
public:
    BoatImuSourceArbitrator()
        : source_(BoatImuSource::None), device_id_(""), last_update_us_(0),
          timeout_us_(default_boatimu_source_timeout_us), has_source_(false) {}

    void reset() {
        source_ = BoatImuSource::None;
        device_id_ = "";
        last_update_us_ = 0;
        has_source_ = false;
    }

    void set_timeout_us(uint64_t timeout_us) { timeout_us_ = timeout_us; }
    uint64_t timeout_us() const { return timeout_us_; }

    BoatImuSource source() const { return source_; }
    const char* device_id() const { return device_id_; }
    uint64_t last_update_us() const { return last_update_us_; }
    bool has_source() const { return has_source_; }

    bool is_stale(uint64_t now_us) const {
        if (!has_source_ || last_update_us_ == 0) {
            return false;
        }
        return now_us >= last_update_us_ && (now_us - last_update_us_) > timeout_us_;
    }

    bool poll_timeout(uint64_t now_us) {
        if (!is_stale(now_us)) {
            return false;
        }
        reset();
        return true;
    }

    bool can_accept(const BoatImuSourceTag& candidate, uint64_t now_us) const {
        if (candidate.source == BoatImuSource::None) {
            return false;
        }
        if (!has_source_ || is_stale(now_us)) {
            return true;
        }

        const uint8_t candidate_priority = boatimu_source_priority(candidate.source);
        const uint8_t current_priority = boatimu_source_priority(source_);

        if (candidate_priority < current_priority) {
            return true;
        }
        if (candidate_priority > current_priority) {
            return false;
        }

        return candidate.source == source_ && boatimu_device_id_equal(candidate.device_id, device_id_);
    }

    bool accept(const BoatImuSourceTag& candidate, uint64_t now_us) {
        if (!can_accept(candidate, now_us)) {
            return false;
        }
        source_ = candidate.source;
        device_id_ = candidate.device_id ? candidate.device_id : "";
        last_update_us_ = now_us;
        has_source_ = true;
        return true;
    }

private:
    BoatImuSource source_;
    const char* device_id_;
    uint64_t last_update_us_;
    uint64_t timeout_us_;
    bool has_source_;
};

} // namespace pypilot_boatimu
