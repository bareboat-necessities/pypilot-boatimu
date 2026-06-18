#include <cassert>
#include <cmath>

#include "pypilot_boatimu.hpp"

int main() {
    using namespace pypilot_boatimu;

    BoatImuSourceArbitrator arb;
    assert(!arb.has_source());
    assert(arb.accept(BoatImuSourceTag(BoatImuSource::RTIMULib2, "imu0"), 1000000));
    assert(arb.has_source());
    assert(arb.source() == BoatImuSource::RTIMULib2);

    assert(!arb.accept(BoatImuSourceTag(BoatImuSource::RTIMULib2, "imu1"), 1100000));
    assert(arb.source() == BoatImuSource::RTIMULib2);
    assert(boatimu_device_id_equal(arb.device_id(), "imu0"));

    assert(arb.accept(BoatImuSourceTag(BoatImuSource::OceanImu, "ocean0"), 1200000));
    assert(arb.source() == BoatImuSource::OceanImu);
    assert(boatimu_device_id_equal(arb.device_id(), "ocean0"));

    arb.set_timeout_us(100);
    assert(!arb.poll_timeout(1200050));
    assert(arb.poll_timeout(1200101));
    assert(!arb.has_source());

    BoatImuAlignment<double> alignment;
    alignment.heading_offset_deg = 12.0;
    alignment.yaw_offset_deg = 5.0;
    assert(std::fabs(align_heading_deg(350.0, alignment) - 7.0) < 1e-9);

    HeadingSample<double> heading;
    heading.has_heading = true;
    heading.heading_deg = 350.0;
    HeadingSample<double> aligned_heading = apply_alignment(heading, alignment);
    assert(std::fabs(aligned_heading.heading_deg - 7.0) < 1e-9);

    FusedAttitudeSample<double> attitude;
    attitude.has_euler = true;
    attitude.roll_deg = 1.0;
    attitude.pitch_deg = 2.0;
    attitude.yaw_deg = 350.0;
    alignment.roll_offset_deg = -3.0;
    alignment.pitch_offset_deg = 4.0;
    FusedAttitudeSample<double> aligned_attitude = apply_alignment(attitude, alignment);
    assert(std::fabs(aligned_attitude.roll_deg - -2.0) < 1e-9);
    assert(std::fabs(aligned_attitude.pitch_deg - 6.0) < 1e-9);
    assert(std::fabs(aligned_attitude.yaw_deg - 7.0) < 1e-9);

    FusedAttitudeSample<double> qsample;
    qsample.has_quaternion = true;
    qsample.q = Quaternion<double>(1.0, 0.0, 0.0, 0.0);
    BoatImuAlignment<double> yaw_alignment;
    yaw_alignment.yaw_offset_deg = 90.0;
    FusedAttitudeSample<double> aligned_q = apply_alignment(qsample, yaw_alignment);
    assert(quaternion_is_valid(aligned_q.q));
    assert(std::fabs(quaternion_norm_squared(aligned_q.q) - 1.0) < 1e-9);

    return 0;
}
