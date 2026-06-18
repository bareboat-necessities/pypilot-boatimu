#include <cassert>
#include <cmath>

#include "pypilot_boatimu.hpp"

int main() {
    using namespace pypilot_boatimu;

    BoatImuRuntimeConfig<double> config;
    config.alignment.heading_offset_deg = 10.0;
    BoatImuRuntime<double> runtime(config);
    runtime.set_source_timeout_us(100);

    HeadingSample<double> low_priority;
    low_priority.time_us = 1000;
    low_priority.has_heading = true;
    low_priority.heading_deg = 15.0;
    low_priority.source_name = "heading";
    low_priority.device_id = "h0";

    assert(runtime.update_heading(low_priority, BoatImuSourceTag(BoatImuSource::HeadingOnly, "h0")));
    assert(runtime.state().has_heading);
    assert(std::fabs(runtime.state().heading.heading_deg - 25.0) < 1e-9);
    assert(runtime.state().source == BoatImuSource::HeadingOnly);

    FusedAttitudeSample<double> fused;
    fused.time_us = 1050;
    fused.has_euler = true;
    fused.roll_deg = 1.0;
    fused.pitch_deg = 2.0;
    fused.yaw_deg = 30.0;
    fused.source_name = "ocean";
    fused.device_id = "ocean0";

    assert(runtime.update_fused_attitude(fused, BoatImuSourceTag(BoatImuSource::OceanImu, "ocean0")));
    assert(runtime.state().has_fused_attitude);
    assert(runtime.state().has_heading);
    assert(std::fabs(runtime.state().fused_attitude.yaw_deg - 40.0) < 1e-9);
    assert(std::fabs(runtime.state().heading.heading_deg - 40.0) < 1e-9);
    assert(runtime.state().source == BoatImuSource::OceanImu);

    HeadingSample<double> rejected;
    rejected.time_us = 1060;
    rejected.has_heading = true;
    rejected.heading_deg = 90.0;
    assert(!runtime.update_heading(rejected, BoatImuSourceTag(BoatImuSource::HeadingOnly, "h1")));
    assert(runtime.state().source == BoatImuSource::OceanImu);

    assert(!runtime.poll_timeout(1100));
    assert(runtime.poll_timeout(1200));
    assert(!runtime.state().has_heading);
    assert(runtime.state().source == BoatImuSource::None);

    CalibratedImuSample<double> imu;
    imu.time_us = 2000;
    imu.has_accel = true;
    imu.accel_mps2 = Vector3<double>(1.0, 0.0, 0.0);
    BoatImuAlignment<double> align90;
    align90.yaw_offset_deg = 90.0;
    runtime.set_alignment(align90);
    assert(runtime.update_calibrated_imu(imu, BoatImuSourceTag(BoatImuSource::CalibratedImu, "imu0")));
    assert(runtime.state().has_calibrated_imu);
    assert(std::fabs(runtime.state().calibrated_imu.accel_mps2.x) < 1e-9);
    assert(std::fabs(runtime.state().calibrated_imu.accel_mps2.y - 1.0) < 1e-9);

    MarineMotionSample<double> motion;
    motion.time_us = 3000;
    motion.has_heading = true;
    motion.heading_deg = 350.0;
    motion.has_heave = true;
    motion.heave_m = 0.3;
    assert(runtime.update_marine_motion(motion, BoatImuSourceTag(BoatImuSource::OceanImu, "ocean0")));
    assert(runtime.state().has_marine_motion);
    assert(runtime.state().has_heading);
    assert(std::fabs(runtime.state().heading.heading_deg - 80.0) < 1e-9);

    return 0;
}
