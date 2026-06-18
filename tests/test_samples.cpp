#include <cassert>
#include <cmath>

#include "pypilot_boatimu.hpp"

int main() {
    using namespace pypilot_boatimu;

    CalibratedImuSample<double> imu;
    assert(!calibrated_imu_sample_has_data(imu));
    imu.has_accel = true;
    imu.accel_mps2 = Vector3<double>(0.0, 0.0, 9.80665);
    assert(calibrated_imu_sample_has_data(imu));

    FusedAttitudeSample<double> attitude;
    assert(!fused_attitude_sample_has_orientation(attitude));
    attitude.has_quaternion = true;
    attitude.q = Quaternion<double>(1.0, 0.0, 0.0, 0.0);
    assert(fused_attitude_sample_has_orientation(attitude));
    attitude.q = Quaternion<double>(0.0, 0.0, 0.0, 0.0);
    assert(!fused_attitude_sample_has_orientation(attitude));

    HeadingSample<double> heading;
    assert(!heading_sample_is_valid(heading));
    heading.has_heading = true;
    heading.heading_deg = 725.0;
    assert(heading_sample_is_valid(heading));
    assert(std::fabs(normalize_heading_360(heading.heading_deg) - 5.0) < 1e-9);

    MarineMotionSample<double> motion;
    assert(!marine_motion_sample_has_data(motion));
    motion.has_heave = true;
    motion.heave_m = 0.25;
    assert(marine_motion_sample_has_data(motion));

    return 0;
}
