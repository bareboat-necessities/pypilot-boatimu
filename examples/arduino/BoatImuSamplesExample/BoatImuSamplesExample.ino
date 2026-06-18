#include <Arduino.h>
#include <pypilot_boatimu.hpp>

void setup() {
  Serial.begin(115200);
  delay(100);

  pypilot_boatimu::HeadingSample<float> heading;
  heading.has_heading = true;
  heading.heading_deg = 361.5f;
  heading.source_name = "arduino-example";
  heading.device_id = "simulated";

  Serial.print("heading valid=");
  Serial.println(pypilot_boatimu::heading_sample_is_valid(heading) ? "true" : "false");
  Serial.print("heading normalized=");
  Serial.println(pypilot_boatimu::normalize_heading_360(heading.heading_deg));
}

void loop() {
}
