#pragma once

#include <string.h>

#include "pypilot_boatimu/runtime.hpp"
#include <pypilot_data_model.hpp>

namespace pypilot_boatimu {

static const double boatimu_g_std_mps2 = 9.80665;

template<typename Real>
inline pypilot_data_model::Vec3<Real> to_data_model_vec3(const Vector3<Real>& v) {
    pypilot_data_model::Vec3<Real> out;
    out.x = v.x;
    out.y = v.y;
    out.z = v.z;
    return out;
}

template<typename Real>
inline pypilot_data_model::Quat<Real> to_data_model_quat(const Quaternion<Real>& q) {
    pypilot_data_model::Quat<Real> out;
    out.w = q.w;
    out.x = q.x;
    out.y = q.y;
    out.z = q.z;
    return out;
}

template<typename Real>
inline pypilot_data_model::Vec3<Real> accel_mps2_to_g(const Vector3<Real>& v) {
    const Real inv_g = static_cast<Real>(1.0 / boatimu_g_std_mps2);
    return to_data_model_vec3(Vector3<Real>(v.x * inv_g, v.y * inv_g, v.z * inv_g));
}

template<typename Real>
inline pypilot_data_model::Vec3<Real> gyro_rps_to_deg_s(const Vector3<Real>& v) {
    const Real k = static_cast<Real>(180.0 / 3.141592653589793238462643383279502884);
    return to_data_model_vec3(Vector3<Real>(v.x * k, v.y * k, v.z * k));
}

inline void copy_device_id(char* dst, size_t dst_size, const char* src) {
    if (dst_size == 0) {
        return;
    }
    if (!src) {
        src = "";
    }
    strncpy(dst, src, dst_size - 1);
    dst[dst_size - 1] = 0;
}

template<typename Real>
class BoatImuDataModelWriter {
public:
    explicit BoatImuDataModelWriter(pypilot_data_model::DataModel<Real>& model)
        : model_(&model) {}

    bool write(const BoatImuState<Real>& state) {
        if (!model_) {
            return false;
        }

        bool wrote = false;
        pypilot_data_model::BoatImuData<Real>& imu = model_->imu;
        imu.source_kind.value = static_cast<uint8_t>(state.source);
        imu.last_update_us = state.last_update_us;
        copy_device_id(imu.device_id, sizeof(imu.device_id), state.device_id);

        if (state.has_calibrated_imu) {
            const CalibratedImuSample<Real>& s = state.calibrated_imu;
            if (s.has_accel) {
                imu.accel_g.set(accel_mps2_to_g(s.accel_mps2), s.time_us);
                wrote = true;
            }
            if (s.has_gyro) {
                imu.gyro_deg_s.set(gyro_rps_to_deg_s(s.gyro_rps), s.time_us);
                wrote = true;
            }
            if (s.has_mag) {
                imu.compass_raw.set(to_data_model_vec3(s.mag_uT), s.time_us);
                wrote = true;
            }
        }

        if (state.has_fused_attitude) {
            const FusedAttitudeSample<Real>& s = state.fused_attitude;
            if (s.has_euler) {
                imu.roll_deg.set(s.roll_deg, s.time_us);
                imu.pitch_deg.set(s.pitch_deg, s.time_us);
                imu.heading_deg.set(normalize_heading_360(s.yaw_deg), s.time_us);
                wrote = true;
            }
            if (s.has_quaternion) {
                imu.fusion_q_pose.set(to_data_model_quat(s.q), s.time_us);
                wrote = true;
            }
        }

        if (state.has_heading) {
            const HeadingSample<Real>& s = state.heading;
            if (s.has_heading) {
                imu.heading_deg.set(normalize_heading_360(s.heading_deg), s.time_us);
                wrote = true;
            }
            if (s.has_heading_rate) {
                imu.heading_rate_deg_s.set(s.heading_rate_dps, s.time_us);
                wrote = true;
            }
        }

        if (state.has_marine_motion) {
            const MarineMotionSample<Real>& s = state.marine_motion;
            if (s.has_heel) {
                imu.heel_deg.set(s.heel_deg, s.time_us);
                wrote = true;
            }
            if (s.has_trim) {
                imu.pitch_deg.set(s.trim_deg, s.time_us);
                wrote = true;
            }
            if (s.has_heading) {
                imu.heading_deg.set(normalize_heading_360(s.heading_deg), s.time_us);
                wrote = true;
            }
            if (s.has_rate_of_turn) {
                imu.heading_rate_deg_s.set(s.rate_of_turn_dps, s.time_us);
                wrote = true;
            }
        }

        return wrote;
    }

private:
    pypilot_data_model::DataModel<Real>* model_;
};

} // namespace pypilot_boatimu
