#include <cassert>
#include <cmath>

#include "pypilot_boatimu.hpp"

int main() {
    using namespace pypilot_boatimu;

    GenericCalibratedSampleAdapter<double> calibrated("serial", "imu0");
    Vector3<double> accel(0.0, 0.0, 9.80665);
    Vector3<double> gyro(0.0, 0.0, 0.1);
    CalibratedImuSample<double> cs = calibrated.make_sample(1000, &accel, &gyro, 0);
    assert(cs.has_accel);
    assert(cs.has_gyro);
    assert(!cs.has_mag);
    assert(calibrated_imu_sample_has_data(cs));
    assert(calibrated.source_tag().source == BoatImuSource::CalibratedImu);

    GenericFusedAttitudeAdapter<double> fused(BoatImuSource::TrustedFused, "trusted", "ahrs0");
    FusedAttitudeSample<double> fs = fused.make_full_sample(2000, 1.0, 2.0, 3.0, Quaternion<double>(1.0, 0.0, 0.0, 0.0));
    assert(fs.has_euler);
    assert(fs.has_quaternion);
    assert(fused_attitude_sample_has_orientation(fs));
    assert(fused.source_tag().source == BoatImuSource::TrustedFused);

    RTIMULib2AdapterStub<double> rtimulib("rt0");
    FusedAttitudeSample<double> rs = rtimulib.make_fused_sample(3000, 4.0, 5.0, 6.0, 0);
    assert(rs.has_euler);
    assert(rtimulib.source_tag().source == BoatImuSource::RTIMULib2);

    OceanImuAdapterStub<double> ocean("ocean0");
    double heave = 0.25;
    MarineMotionSample<double> ms = ocean.make_marine_motion_sample(4000, 7.0, -1.0, 280.0, 0.5, &heave, 0, 0);
    assert(ms.has_heel);
    assert(ms.has_trim);
    assert(ms.has_heading);
    assert(ms.has_rate_of_turn);
    assert(ms.has_heave);
    assert(marine_motion_sample_has_data(ms));
    assert(ocean.source_tag().source == BoatImuSource::OceanImu);

    HeadingSample<double> replay_samples[2];
    replay_samples[0].time_us = 1;
    replay_samples[0].has_heading = true;
    replay_samples[0].heading_deg = 10.0;
    replay_samples[1].time_us = 2;
    replay_samples[1].has_heading = true;
    replay_samples[1].heading_deg = 20.0;

    ReplayAdapter<HeadingSample<double> > replay(replay_samples, 2);
    HeadingSample<double> out;
    assert(replay.has_next());
    assert(replay.next(out));
    assert(out.heading_deg == 10.0);
    assert(replay.next(out));
    assert(out.heading_deg == 20.0);
    assert(!replay.next(out));
    replay.rewind();
    assert(replay.next(out));
    assert(out.heading_deg == 10.0);

    return 0;
}
