#include <iostream>

#include "pypilot_boatimu.hpp"

int main() {
    pypilot_boatimu::FusedAttitudeSample<double> sample;
    sample.time_us = 1000000;
    sample.has_quaternion = true;
    sample.q = pypilot_boatimu::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
    sample.source_name = "example";
    sample.device_id = "simulated";

    std::cout << "BoatIMU fused attitude valid="
              << pypilot_boatimu::fused_attitude_sample_has_orientation(sample)
              << " source=" << sample.source_name
              << " device=" << sample.device_id
              << '\n';
    return 0;
}
