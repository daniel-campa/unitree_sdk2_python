// The pendulum rig's hinge encoder for go2/pole_encoder.py: a 600 P/R A/B quadrature encoder on
// an ESP32-S3, counted by its pulse counter and sent over USB 500 times a second as
// "<micros> <angle in radians>" lines. Needs the Arduino ESP32 core 3.0 or later.
// Untested on hardware, and not yet compiled.
//
// The count starts from wherever the shaft is at power-up: go2/run_policy.py zeroes it on the
// pole hanging still before each walk.
//
// Wiring: encoder red to 5V, black to GND, white (A) to GPIO 4, green (B) to GPIO 5. A and B
// only pull low (open collector), so each needs a 2.2-4.7 kOhm pull-up to 3V3. If A or B sits
// near 5 V with the encoder powered and nothing else connected, the encoder drives it high
// itself: connecting it would damage the ESP32-S3, so it needs a level shifter instead.
//
// Arduino IDE: board "ESP32S3 Dev Module", USB CDC On Boot "Enabled", and the cable in the
// port labelled USB, which appears as /dev/ttyACM0.
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

const int PIN_A = 4;
const int PIN_B = 5;
const int COUNTS_PER_TURN = 2400;  // both edges of A and B: 4 per pulse
const int COUNTER_LIMIT = 30000;
const TickType_t PERIOD = pdMS_TO_TICKS(2);

pcnt_unit_handle_t counter = NULL;

void setup() {
  Serial.begin(500000);  // the USB port ignores it; the UART port needs this much

  pcnt_unit_config_t unit_config = {};
  unit_config.low_limit = -COUNTER_LIMIT;
  unit_config.high_limit = COUNTER_LIMIT;
  unit_config.flags.accum_count = 1;  // with watch points at the limits, carries the count past them
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &counter));
  pcnt_glitch_filter_config_t filter = {};
  filter.max_glitch_ns = 1000;
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(counter, &filter));

  pcnt_chan_config_t a_config = {};
  a_config.edge_gpio_num = PIN_A;
  a_config.level_gpio_num = PIN_B;
  pcnt_channel_handle_t a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(counter, &a_config, &a));
  pcnt_chan_config_t b_config = {};
  b_config.edge_gpio_num = PIN_B;
  b_config.level_gpio_num = PIN_A;
  pcnt_channel_handle_t b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(counter, &b_config, &b));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(counter, -COUNTER_LIMIT));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(counter, COUNTER_LIMIT));
  gpio_pullup_en((gpio_num_t)PIN_A);  // weak; the external pull-ups give clean edges
  gpio_pullup_en((gpio_num_t)PIN_B);
  ESP_ERROR_CHECK(pcnt_unit_enable(counter));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(counter));
  ESP_ERROR_CHECK(pcnt_unit_start(counter));
}

void loop() {
  static TickType_t wake = xTaskGetTickCount();
  vTaskDelayUntil(&wake, PERIOD);
  unsigned long now = micros();
  int count = 0;
  if (pcnt_unit_get_count(counter, &count) != ESP_OK) {
    return;  // a skipped reading; go2/check_pole.py reports the gap
  }
  int turn = ((count % COUNTS_PER_TURN) + COUNTS_PER_TURN) % COUNTS_PER_TURN;
  char line[32];
  int length = snprintf(line, sizeof(line), "%lu %.5f\n", now, turn * (TWO_PI / COUNTS_PER_TURN));
  // A whole line or none: while nothing reads the port, a cut line would join the next into a wrong reading.
  if (Serial.availableForWrite() >= length) {
    Serial.write(line, length);
  }
}
