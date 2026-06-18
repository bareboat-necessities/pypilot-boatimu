#pragma once

#include "pypilot_boatimu/adapters/generic_calibrated_sample_adapter.hpp"
#include "pypilot_boatimu/adapters/generic_fused_attitude_adapter.hpp"

namespace pypilot_boatimu {

template<typename Real>
class OceanImuAdapterStub {
public:
    explicit OceanImuAdapterStub(const char* device_id = "ocean-imu")
        : calibrated_("ocean_imu", device_id),
          fused_(BoatImuSource::OceanImu, "ocean_imu", device_id),
          device_id_(device_id ? device_id : "ocean-imu") {}

    BoatImuSourceTag source_tag() const {
        return BoatImuSourceTag(BoatImuSource::OceanImu, device_id_);
    }

    CalibratedImuSample<Real> make_calibrated_sample(uint64_t time_us,
                                                     const Vector3<Real>* accel_mps2,
                                                     const Vector3<Real>* gyro_rps,
                                                     const Vector3<Real>* mag_uT) const {
        return calibrated_.make_sample(time_us, accel_mps2, gyro_rps, mag_uT);
    }

    FusedAttitudeSample<Real> make_fused_sample(uint64_t time_us,
                                                Real roll_deg,
                                                Real pitch_deg,
                                                Real yaw_deg,
                                                const Quaternion<Real>* q) const {
        if (q) {
            return fused_.make_full_sample(time_us, roll_deg, pitch_deg, yaw_deg, *q);
        }
        return fused_.make_euler_sample(time_us, roll_deg, pitch_deg, yaw_deg);
    }

    MarineMotionSample<Real> make_marine_motion_sample(uint64_t time_us,
                                                       Real heel_deg,
                                                       Real trim_deg,
                                                       Real heading_deg,
                                                       Real rate_of_turn_dps,
                                                       const Real* heave_m,
                                                       const Real* surge_m,
                                                       const Real* sway_m) const {
        MarineMotionSample<Real> out;
        out.time_us = time_us;
        out.heel_deg = heel_deg;
        out.trim_deg = trim_deg;
        out.heading_deg = heading_deg;
        out.rate_of_turn_dps = rate_of_turn_dps;
        out.has_heel = true;
        out.has_trim = true;
        out.has_heading = true;
        out.has_rate_of_turn = true;
        if (heave_m) {
            out.heave_m = *heave_m;
            out.has_heave = true;
        }
        if (surge_m) {
            out.surge_m = *surge_m;
            out.has_surge = true;
        }
        if (sway_m) {
            out.sway_m = *sway_m;
            out.has_sway = true;
        }
        out.source_name = "ocean_imu";
        out.device_id = device_id_;
        return out;
    }

private:
    GenericCalibratedSampleAdapter<Real> calibrated_;
    GenericFusedAttitudeAdapter<Real> fused_;
    const char* device_id_;
};

} // namespace pypilot_boatimu
