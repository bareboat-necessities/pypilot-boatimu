#include <cassert>
#include <cmath>
#include <cstring>

#include "pypilot_boatimu.hpp"
#include "pypilot_boatimu/data_model_writer.hpp"

int main() {
    using namespace pypilot_boatimu;

    pypilot_data_model::DataModel<double> model;
    BoatImuRuntime<double> runtime;

    FusedAttitudeSample<double> fused;
    fused.time_us = 1000;
    fused.has_euler = true;
    fused.roll_deg = 3.0;
    fused.pitch_deg = -2.0;
    fused.yaw_deg = 361.0;
    fused.has_quaternion = true;
    fused.q = Quaternion<double>(1.0, 0.0, 0.0, 0.0);
    assert(runtime.update_fused_attitude(fused, BoatImuSourceTag(BoatImuSource::OceanImu, "ocean0")));

    HeadingSample<double> heading;
    heading.time_us = 1100;
    heading.has_heading = true;
    heading.heading_deg = 42.0;
    heading.has_heading_rate = true;
    heading.heading_rate_dps = 1.25;
    assert(runtime.update_heading(heading, BoatImuSourceTag(BoatImuSource::OceanImu, "ocean0")));

    CalibratedImuSample<double> imu;
    imu.time_us = 1200;
    imu.has_accel = true;
    imu.accel_mps2 = Vector3<double>(0.0, 0.0, boatimu_g_std_mps2);
    imu.has_gyro = true;
    imu.gyro_rps = Vector3<double>(0.0, 0.0, 3.14159265358979323846 / 2.0);
    imu.has_mag = true;
    imu.mag_uT = Vector3<double>(12.0, 13.0, 14.0);
    assert(runtime.update_calibrated_imu(imu, BoatImuSourceTag(BoatImuSource::OceanImu, "ocean0")));

    MarineMotionSample<double> motion;
    motion.time_us = 1300;
    motion.has_heel = true;
    motion.heel_deg = 7.0;
    motion.has_trim = true;
    motion.trim_deg = -1.5;
    motion.has_rate_of_turn = true;
    motion.rate_of_turn_dps = 0.5;
    assert(runtime.update_marine_motion(motion, BoatImuSourceTag(BoatImuSource::OceanImu, "ocean0")));

    BoatImuDataModelWriter<double> writer(model);
    assert(writer.write(runtime.state()));

    assert(model.imu.source_kind.value == static_cast<uint8_t>(BoatImuSource::OceanImu));
    assert(std::strcmp(model.imu.device_id, "ocean0") == 0);
    assert(model.imu.last_update_us == 1300);

    assert(model.imu.roll_deg.valid);
    assert(std::fabs(model.imu.roll_deg.value - 3.0) < 1e-9);
    assert(model.imu.pitch_deg.valid);
    assert(std::fabs(model.imu.pitch_deg.value - -1.5) < 1e-9);
    assert(model.imu.heel_deg.valid);
    assert(std::fabs(model.imu.heel_deg.value - 7.0) < 1e-9);
    assert(model.imu.heading_deg.valid);
    assert(std::fabs(model.imu.heading_deg.value - 42.0) < 1e-9);
    assert(model.imu.heading_rate_deg_s.valid);
    assert(std::fabs(model.imu.heading_rate_deg_s.value - 0.5) < 1e-9);

    assert(model.imu.fusion_q_pose.valid);
    assert(std::fabs(model.imu.fusion_q_pose.value.w - 1.0) < 1e-9);

    assert(model.imu.accel_g.valid);
    assert(std::fabs(model.imu.accel_g.value.z - 1.0) < 1e-9);
    assert(model.imu.gyro_deg_s.valid);
    assert(std::fabs(model.imu.gyro_deg_s.value.z - 90.0) < 1e-9);
    assert(model.imu.compass_raw.valid);
    assert(std::fabs(model.imu.compass_raw.value.x - 12.0) < 1e-9);

    return 0;
}
