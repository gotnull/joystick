// Compile: arduino-cli compile --fqbn esp32:esp32:esp32 .
// Upload: arduino-cli upload -p /dev/cu.usbserial-120 --fqbn esp32:esp32:esp32 .
// Monitor: arduino-cli monitor -p /dev/cu.usbserial-120 --config 115200

#include <USBComposite.h>
#define PIN_X PA0
#define PIN_Y PA1
#define BUTTONS 6 // Number of buttons

uint8_t button_pins[BUTTONS] = {PB11, PB10, PB1, PB0, PA7, PA6};
// mapping to gamepad buttons
uint8_t button_mapping[BUTTONS] = {9, 13, 15, 14, 10, 16};

// Calibration values
// Check initial values without map-constrain and tweak until satisfied
const int16_t x_offset = 2300 / 2;
const int16_t x_low = 800 - x_offset;
const int16_t x_high = 3731 - x_offset;

const int16_t y_offset = 1750 / 2;
const int16_t y_low = 520 - y_offset;
const int16_t y_high = 3338 - y_offset;

USBXBox360 XBox360;

void setup()
{
  pinMode(PIN_X, INPUT_ANALOG);
  pinMode(PIN_Y, INPUT_ANALOG);
  for (int i = 0; i < BUTTONS; i++)
  {
    pinMode(button_pins[i], INPUT_PULLUP);
  }
  // Send manually reports every run and not on every change
  XBox360.setManualReportMode(true);
  XBox360.begin();
  delay(1000);
}

void loop()
{
  int16_t x_val = analogRead(PIN_X);
  int16_t y_val = analogRead(PIN_Y);

  // comment these out to disable scaling for calibration
  x_val = map(constrain(x_val - x_offset, x_low, x_high), x_low, x_high, -0x7fff, 0x7fff);
  y_val = map(constrain(y_val - y_offset, y_low, y_high), y_low, y_high, -0x7fff, 0x7fff);

  XBox360.X(x_val);
  XBox360.Y(y_val);

  // Map sliders to half y axes for racing games
  XBox360.sliderRight(constrain(y_val >> 7, 0, 255));
  XBox360.sliderLeft(constrain(-(y_val >> 7), 0, 255));

  for (int i = 0; i < BUTTONS; i++)
  {
    XBox360.button(button_mapping[i], !digitalRead(button_pins[i]));
  }

  XBox360.send(); // Send report
  delay(1);
}