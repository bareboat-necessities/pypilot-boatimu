#pragma once

#include <stdint.h>
#include <math.h>

namespace pypilot_boatimu {

template<typename Real>
struct Vector3 {
    Real x;
    Real y;
    Real z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(Real x_, Real y_, Real z_) : x(x_), y(y_), z(z_) {}
};

template<typename Real>
struct Quaternion {
    Real w;
    Real x;
    Real y;
    Real z;

    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(Real w_, Real x_, Real y_, Real z_) : w(w_), x(x_), y(y_), z(z_) {}
};

enum class BoatImuSourceKind : uint8_t {
    None = 0,
    CalibratedImu = 1,
    FusedAttitude = 2,
    Heading = 3,
    MarineMotion = 4,
    Replay = 5
};

template<typename Real>
struct CalibratedImuSample {
    uint64_t time_us;
    Vector3<Real> accel_mps2;
    Vector3<Real> gyro_rps;
    Vector3<Real> mag_uT;
    bool has_accel;
    bool has_gyro;
    bool has_mag;
    const char* source_name;
    const char* device_id;

    CalibratedImuSample()
        : time_us(0), accel_mps2(), gyro_rps(), mag_uT(),
          has_accel(false), has_gyro(false), has_mag(false),
          source_name(""), device_id("") {}
};

template<typename Real>
struct FusedAttitudeSample {
    uint64_t time_us;
    Quaternion<Real> q;
    Real roll_deg;
    Real pitch_deg;
    Real yaw_deg;
    bool has_quaternion;
    bool has_euler;
    const char* source_name;
    const char* device_id;

    FusedAttitudeSample()
        : time_us(0), q(), roll_deg(0), pitch_deg(0), yaw_deg(0),
          has_quaternion(false), has_euler(false),
          source_name(""), device_id("") {}
};

template<typename Real>
struct HeadingSample {
    uint64_t time_us;
    Real heading_deg;
    Real heading_rate_dps;
    bool has_heading;
    bool has_heading_rate;
    const char* source_name;
    const char* device_id;

    HeadingSample()
        : time_us(0), heading_deg(0), heading_rate_dps(0),
          has_heading(false), has_heading_rate(false),
          source_name(""), device_id("") {}
};

template<typename Real>
struct MarineMotionSample {
    uint64_t time_us;
    Real heel_deg;
    Real trim_deg;
    Real heading_deg;
    Real rate_of_turn_dps;
    Real heave_m;
    Real surge_m;
    Real sway_m;
    bool has_heel;
    bool has_trim;
    bool has_heading;
    bool has_rate_of_turn;
    bool has_heave;
    bool has_surge;
    bool has_sway;
    const char* source_name;
    const char* device_id;

    MarineMotionSample()
        : time_us(0), heel_deg(0), trim_deg(0), heading_deg(0),
          rate_of_turn_dps(0), heave_m(0), surge_m(0), sway_m(0),
          has_heel(false), has_trim(false), has_heading(false),
          has_rate_of_turn(false), has_heave(false), has_surge(false), has_sway(false),
          source_name(""), device_id("") {}
};

template<typename Real>
inline bool finite_value(Real v) {
    return isfinite(static_cast<double>(v)) != 0;
}

template<typename Real>
inline bool finite_vector(const Vector3<Real>& v) {
    return finite_value(v.x) && finite_value(v.y) && finite_value(v.z);
}

template<typename Real>
inline Real quaternion_norm_squared(const Quaternion<Real>& q) {
    return q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
}

template<typename Real>
inline bool finite_quaternion(const Quaternion<Real>& q) {
    return finite_value(q.w) && finite_value(q.x) && finite_value(q.y) && finite_value(q.z);
}

template<typename Real>
inline bool quaternion_is_valid(const Quaternion<Real>& q) {
    if (!finite_quaternion(q)) {
        return false;
    }
    const Real n2 = quaternion_norm_squared(q);
    return n2 > static_cast<Real>(0.25) && n2 < static_cast<Real>(4.0);
}

template<typename Real>
inline Real normalize_heading_360(Real heading_deg) {
    while (heading_deg < static_cast<Real>(0)) {
        heading_deg += static_cast<Real>(360);
    }
    while (heading_deg >= static_cast<Real>(360)) {
        heading_deg -= static_cast<Real>(360);
    }
    return heading_deg;
}

template<typename Real>
inline bool calibrated_imu_sample_has_data(const CalibratedImuSample<Real>& s) {
    return (s.has_accel && finite_vector(s.accel_mps2)) ||
           (s.has_gyro && finite_vector(s.gyro_rps)) ||
           (s.has_mag && finite_vector(s.mag_uT));
}

template<typename Real>
inline bool fused_attitude_sample_has_orientation(const FusedAttitudeSample<Real>& s) {
    const bool q_ok = s.has_quaternion && quaternion_is_valid(s.q);
    const bool euler_ok = s.has_euler && finite_value(s.roll_deg) && finite_value(s.pitch_deg) && finite_value(s.yaw_deg);
    return q_ok || euler_ok;
}

template<typename Real>
inline bool heading_sample_is_valid(const HeadingSample<Real>& s) {
    if (!s.has_heading || !finite_value(s.heading_deg)) {
        return false;
    }
    if (s.has_heading_rate && !finite_value(s.heading_rate_dps)) {
        return false;
    }
    return true;
}

template<typename Real>
inline bool marine_motion_sample_has_data(const MarineMotionSample<Real>& s) {
    return (s.has_heel && finite_value(s.heel_deg)) ||
           (s.has_trim && finite_value(s.trim_deg)) ||
           (s.has_heading && finite_value(s.heading_deg)) ||
           (s.has_rate_of_turn && finite_value(s.rate_of_turn_dps)) ||
           (s.has_heave && finite_value(s.heave_m)) ||
           (s.has_surge && finite_value(s.surge_m)) ||
           (s.has_sway && finite_value(s.sway_m));
}

} // namespace pypilot_boatimu
