#pragma once

#include "pypilot_boatimu/alignment.hpp"
#include "pypilot_boatimu/source_policy.hpp"

namespace pypilot_boatimu {

template<typename Real>
struct BoatImuRuntimeConfig {
    uint64_t source_timeout_us;
    BoatImuAlignment<Real> alignment;

    BoatImuRuntimeConfig()
        : source_timeout_us(default_boatimu_source_timeout_us), alignment() {}
};

template<typename Real>
struct BoatImuState {
    uint64_t last_update_us;
    BoatImuSource source;
    const char* device_id;

    CalibratedImuSample<Real> calibrated_imu;
    FusedAttitudeSample<Real> fused_attitude;
    HeadingSample<Real> heading;
    MarineMotionSample<Real> marine_motion;

    bool has_calibrated_imu;
    bool has_fused_attitude;
    bool has_heading;
    bool has_marine_motion;

    BoatImuState()
        : last_update_us(0), source(BoatImuSource::None), device_id(""),
          calibrated_imu(), fused_attitude(), heading(), marine_motion(),
          has_calibrated_imu(false), has_fused_attitude(false),
          has_heading(false), has_marine_motion(false) {}
};

template<typename Real>
class BoatImuRuntime {
public:
    BoatImuRuntime() : config_(), state_(), arbitrator_() {
        arbitrator_.set_timeout_us(config_.source_timeout_us);
    }

    explicit BoatImuRuntime(const BoatImuRuntimeConfig<Real>& config)
        : config_(config), state_(), arbitrator_() {
        arbitrator_.set_timeout_us(config_.source_timeout_us);
    }

    const BoatImuRuntimeConfig<Real>& config() const { return config_; }
    const BoatImuState<Real>& state() const { return state_; }
    const BoatImuSourceArbitrator& arbitrator() const { return arbitrator_; }

    void set_alignment(const BoatImuAlignment<Real>& alignment) {
        config_.alignment = alignment;
    }

    const BoatImuAlignment<Real>& alignment() const {
        return config_.alignment;
    }

    void set_source_timeout_us(uint64_t timeout_us) {
        config_.source_timeout_us = timeout_us;
        arbitrator_.set_timeout_us(timeout_us);
    }

    void reset() {
        state_ = BoatImuState<Real>();
        arbitrator_.reset();
    }

    bool poll_timeout(uint64_t now_us) {
        if (!arbitrator_.poll_timeout(now_us)) {
            return false;
        }
        state_ = BoatImuState<Real>();
        return true;
    }

    bool update_calibrated_imu(const CalibratedImuSample<Real>& sample,
                               const BoatImuSourceTag& source) {
        if (!calibrated_imu_sample_has_data(sample)) {
            return false;
        }
        if (!arbitrator_.accept(source, sample.time_us)) {
            return false;
        }
        state_.calibrated_imu = apply_alignment(sample, config_.alignment);
        state_.has_calibrated_imu = true;
        update_common(source, sample.time_us);
        return true;
    }

    bool update_fused_attitude(const FusedAttitudeSample<Real>& sample,
                               const BoatImuSourceTag& source) {
        if (!fused_attitude_sample_has_orientation(sample)) {
            return false;
        }
        if (!arbitrator_.accept(source, sample.time_us)) {
            return false;
        }
        state_.fused_attitude = apply_alignment(sample, config_.alignment);
        state_.has_fused_attitude = true;
        if (state_.fused_attitude.has_euler) {
            state_.heading.time_us = sample.time_us;
            state_.heading.heading_deg = state_.fused_attitude.yaw_deg;
            state_.heading.has_heading = true;
            state_.heading.source_name = sample.source_name;
            state_.heading.device_id = sample.device_id;
            state_.has_heading = true;
        }
        update_common(source, sample.time_us);
        return true;
    }

    bool update_heading(const HeadingSample<Real>& sample,
                        const BoatImuSourceTag& source) {
        if (!heading_sample_is_valid(sample)) {
            return false;
        }
        if (!arbitrator_.accept(source, sample.time_us)) {
            return false;
        }
        state_.heading = apply_alignment(sample, config_.alignment);
        state_.has_heading = true;
        update_common(source, sample.time_us);
        return true;
    }

    bool update_marine_motion(const MarineMotionSample<Real>& sample,
                              const BoatImuSourceTag& source) {
        if (!marine_motion_sample_has_data(sample)) {
            return false;
        }
        if (!arbitrator_.accept(source, sample.time_us)) {
            return false;
        }
        state_.marine_motion = apply_alignment(sample, config_.alignment);
        state_.has_marine_motion = true;
        if (state_.marine_motion.has_heading) {
            state_.heading.time_us = sample.time_us;
            state_.heading.heading_deg = state_.marine_motion.heading_deg;
            state_.heading.has_heading = true;
            state_.heading.source_name = sample.source_name;
            state_.heading.device_id = sample.device_id;
            state_.has_heading = true;
        }
        update_common(source, sample.time_us);
        return true;
    }

private:
    void update_common(const BoatImuSourceTag& source, uint64_t time_us) {
        state_.last_update_us = time_us;
        state_.source = source.source;
        state_.device_id = source.device_id ? source.device_id : "";
    }

    BoatImuRuntimeConfig<Real> config_;
    BoatImuState<Real> state_;
    BoatImuSourceArbitrator arbitrator_;
};

} // namespace pypilot_boatimu
