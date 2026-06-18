#pragma once

#include "pypilot_boatimu/runtime.hpp"

namespace pypilot_boatimu {

template<typename Real>
class GenericCalibratedSampleAdapter {
public:
    GenericCalibratedSampleAdapter(const char* source_name = "generic_calibrated",
                                   const char* device_id = "")
        : source_name_(source_name ? source_name : "generic_calibrated"),
          device_id_(device_id ? device_id : "") {}

    BoatImuSourceTag source_tag() const {
        return BoatImuSourceTag(BoatImuSource::CalibratedImu, device_id_);
    }

    CalibratedImuSample<Real> make_sample(uint64_t time_us,
                                          const Vector3<Real>* accel_mps2,
                                          const Vector3<Real>* gyro_rps,
                                          const Vector3<Real>* mag_uT) const {
        CalibratedImuSample<Real> out;
        out.time_us = time_us;
        out.source_name = source_name_;
        out.device_id = device_id_;
        if (accel_mps2) {
            out.accel_mps2 = *accel_mps2;
            out.has_accel = true;
        }
        if (gyro_rps) {
            out.gyro_rps = *gyro_rps;
            out.has_gyro = true;
        }
        if (mag_uT) {
            out.mag_uT = *mag_uT;
            out.has_mag = true;
        }
        return out;
    }

private:
    const char* source_name_;
    const char* device_id_;
};

} // namespace pypilot_boatimu
