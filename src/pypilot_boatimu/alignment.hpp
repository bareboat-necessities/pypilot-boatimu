#pragma once

#include "pypilot_boatimu/samples.hpp"

namespace pypilot_boatimu {

template<typename Real>
struct BoatImuAlignment {
    Real roll_offset_deg;
    Real pitch_offset_deg;
    Real yaw_offset_deg;
    Real heading_offset_deg;

    BoatImuAlignment()
        : roll_offset_deg(0), pitch_offset_deg(0), yaw_offset_deg(0), heading_offset_deg(0) {}
};

template<typename Real>
inline Real boatimu_pi() {
    return static_cast<Real>(3.141592653589793238462643383279502884);
}

template<typename Real>
inline Real degrees_to_radians(Real degrees) {
    return degrees * boatimu_pi<Real>() / static_cast<Real>(180);
}

template<typename Real>
inline Real radians_to_degrees(Real radians) {
    return radians * static_cast<Real>(180) / boatimu_pi<Real>();
}

template<typename Real>
inline Quaternion<Real> quaternion_multiply(const Quaternion<Real>& a, const Quaternion<Real>& b) {
    return Quaternion<Real>(
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);
}

template<typename Real>
inline Quaternion<Real> quaternion_normalized(const Quaternion<Real>& q) {
    const Real n2 = quaternion_norm_squared(q);
    if (n2 <= static_cast<Real>(0)) {
        return Quaternion<Real>();
    }
    const Real inv_n = static_cast<Real>(1) / static_cast<Real>(sqrt(static_cast<double>(n2)));
    return Quaternion<Real>(q.w * inv_n, q.x * inv_n, q.y * inv_n, q.z * inv_n);
}

template<typename Real>
inline Quaternion<Real> quaternion_from_euler_zyx(Real roll_deg, Real pitch_deg, Real yaw_deg) {
    const Real cr = static_cast<Real>(cos(static_cast<double>(degrees_to_radians(roll_deg) * static_cast<Real>(0.5))));
    const Real sr = static_cast<Real>(sin(static_cast<double>(degrees_to_radians(roll_deg) * static_cast<Real>(0.5))));
    const Real cp = static_cast<Real>(cos(static_cast<double>(degrees_to_radians(pitch_deg) * static_cast<Real>(0.5))));
    const Real sp = static_cast<Real>(sin(static_cast<double>(degrees_to_radians(pitch_deg) * static_cast<Real>(0.5))));
    const Real cy = static_cast<Real>(cos(static_cast<double>(degrees_to_radians(yaw_deg) * static_cast<Real>(0.5))));
    const Real sy = static_cast<Real>(sin(static_cast<double>(degrees_to_radians(yaw_deg) * static_cast<Real>(0.5))));

    Quaternion<Real> q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return quaternion_normalized(q);
}

template<typename Real>
inline Real align_heading_deg(Real heading_deg, const BoatImuAlignment<Real>& alignment) {
    return normalize_heading_360(heading_deg + alignment.yaw_offset_deg + alignment.heading_offset_deg);
}

template<typename Real>
inline Quaternion<Real> align_quaternion(const Quaternion<Real>& q, const BoatImuAlignment<Real>& alignment) {
    const Quaternion<Real> mounting = quaternion_from_euler_zyx(
        alignment.roll_offset_deg,
        alignment.pitch_offset_deg,
        alignment.yaw_offset_deg + alignment.heading_offset_deg);
    return quaternion_normalized(quaternion_multiply(mounting, q));
}

template<typename Real>
inline FusedAttitudeSample<Real> apply_alignment(const FusedAttitudeSample<Real>& sample,
                                                 const BoatImuAlignment<Real>& alignment) {
    FusedAttitudeSample<Real> out = sample;
    if (out.has_euler) {
        out.roll_deg += alignment.roll_offset_deg;
        out.pitch_deg += alignment.pitch_offset_deg;
        out.yaw_deg = align_heading_deg(out.yaw_deg, alignment);
    }
    if (out.has_quaternion) {
        out.q = align_quaternion(out.q, alignment);
    }
    return out;
}

template<typename Real>
inline HeadingSample<Real> apply_alignment(const HeadingSample<Real>& sample,
                                           const BoatImuAlignment<Real>& alignment) {
    HeadingSample<Real> out = sample;
    if (out.has_heading) {
        out.heading_deg = align_heading_deg(out.heading_deg, alignment);
    }
    return out;
}

template<typename Real>
inline MarineMotionSample<Real> apply_alignment(const MarineMotionSample<Real>& sample,
                                                const BoatImuAlignment<Real>& alignment) {
    MarineMotionSample<Real> out = sample;
    if (out.has_heel) {
        out.heel_deg += alignment.roll_offset_deg;
    }
    if (out.has_trim) {
        out.trim_deg += alignment.pitch_offset_deg;
    }
    if (out.has_heading) {
        out.heading_deg = align_heading_deg(out.heading_deg, alignment);
    }
    return out;
}

} // namespace pypilot_boatimu
