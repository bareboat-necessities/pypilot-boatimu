#pragma once

#include "pypilot_boatimu/adapters/generic_calibrated_sample_adapter.hpp"
#include "pypilot_boatimu/adapters/generic_fused_attitude_adapter.hpp"

namespace pypilot_boatimu {

template<typename Real>
class RTIMULib2AdapterStub {
public:
    explicit RTIMULib2AdapterStub(const char* device_id = "rtimulib2")
        : calibrated_("rtimulib2", device_id),
          fused_(BoatImuSource::RTIMULib2, "rtimulib2", device_id) {}

    BoatImuSourceTag source_tag() const {
        return BoatImuSourceTag(BoatImuSource::RTIMULib2, calibrated_.source_tag().device_id);
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

private:
    GenericCalibratedSampleAdapter<Real> calibrated_;
    GenericFusedAttitudeAdapter<Real> fused_;
};

} // namespace pypilot_boatimu
