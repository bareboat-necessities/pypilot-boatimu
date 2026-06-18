#pragma once

#include "pypilot_boatimu/runtime.hpp"

namespace pypilot_boatimu {

template<typename Real>
class GenericFusedAttitudeAdapter {
public:
    GenericFusedAttitudeAdapter(BoatImuSource source = BoatImuSource::TrustedFused,
                                const char* source_name = "generic_fused",
                                const char* device_id = "")
        : source_(source),
          source_name_(source_name ? source_name : "generic_fused"),
          device_id_(device_id ? device_id : "") {}

    BoatImuSourceTag source_tag() const {
        return BoatImuSourceTag(source_, device_id_);
    }

    FusedAttitudeSample<Real> make_euler_sample(uint64_t time_us,
                                                Real roll_deg,
                                                Real pitch_deg,
                                                Real yaw_deg) const {
        FusedAttitudeSample<Real> out;
        out.time_us = time_us;
        out.roll_deg = roll_deg;
        out.pitch_deg = pitch_deg;
        out.yaw_deg = yaw_deg;
        out.has_euler = true;
        out.source_name = source_name_;
        out.device_id = device_id_;
        return out;
    }

    FusedAttitudeSample<Real> make_quaternion_sample(uint64_t time_us,
                                                     const Quaternion<Real>& q) const {
        FusedAttitudeSample<Real> out;
        out.time_us = time_us;
        out.q = q;
        out.has_quaternion = true;
        out.source_name = source_name_;
        out.device_id = device_id_;
        return out;
    }

    FusedAttitudeSample<Real> make_full_sample(uint64_t time_us,
                                               Real roll_deg,
                                               Real pitch_deg,
                                               Real yaw_deg,
                                               const Quaternion<Real>& q) const {
        FusedAttitudeSample<Real> out = make_euler_sample(time_us, roll_deg, pitch_deg, yaw_deg);
        out.q = q;
        out.has_quaternion = true;
        return out;
    }

private:
    BoatImuSource source_;
    const char* source_name_;
    const char* device_id_;
};

} // namespace pypilot_boatimu
