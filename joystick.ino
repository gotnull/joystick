// Compile: arduino-cli compile --fqbn esp32:esp32:ttgo-t1 .
// Upload: arduino-cli upload -p /dev/cu.usbserial-0206671E --fqbn esp32:esp32:ttgo-t1
// Monitor: arduino-cli monitor -p /dev/cu.usbserial-0206671E --config 115200

// Testers:
// https://hardwaretester.com/gamepad
// https://www.onlinemictest.com/controller-tester/

#include <BleGamepad.h>
#include <TFT_eSPI.h>

/*
ESP32-BLE-Gamepad 0.5.4
NimBLE-Arduino    1.4.2
TFT_eSPI          2.5.43
SPI               2.0.0
FS                2.0.0
SPIFFS            2.0.0
*/

// Analog pins for joystick input
#define PIN_X 25 // ESP32 ADC pin for joystick X-axis
#define PIN_Y 26 // ESP32 ADC pin for joystick Y-axis

// Configuration constants
#define BUTTONS 6        // Number of physical buttons
#define CIRCLE_RADIUS 30 // Radius of the circle representing joystick range
#define CENTER_X 120     // Center X position on the TFT
#define CENTER_Y 67      // Center Y position on the TFT
#define STATUS_Y 26      // Y position for the connection status text

// Calibration values for joystick
const int16_t x_offset = 1936 / 2;
const int16_t x_low = 800 - x_offset;
const int16_t x_high = 3731 - x_offset;

const int16_t y_offset = 1444 / 2;
const int16_t y_low = 520 - y_offset;
const int16_t y_high = 3338 - y_offset;

// Button configuration
uint8_t button_pins[BUTTONS] = {32, 33, 25, 26, 27, 14}; // Define button pins for ESP32
uint8_t button_mapping[BUTTONS] = {1, 2, 3, 4, 5, 6};    // Mapping buttons 1 to 6

// TFT Setup
TFT_eSPI tft = TFT_eSPI();

// Initialize BleGamepad
BleGamepad bleGamepad("Gotnull's Gamepad", "gotnull", 100); // Name, Manufacturer, Battery Level
BleGamepadConfiguration bleGamepadConfig;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE Gamepad...");

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Waiting...", tft.width() / 2, STATUS_Y, 4);

  // Initialize joystick and button pins
  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);

  for (int i = 0; i < BUTTONS; i++)
  {
    pinMode(button_pins[i], INPUT_PULLUP);
  }

  // Configure BLE Gamepad
  bleGamepadConfig.setAutoReport(false);                        // Set to manual report mode
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK); // Controller type
  bleGamepadConfig.setButtonCount(BUTTONS);                     // Set number of buttons
  bleGamepadConfig.setHatSwitchCount(0);                        // No hat switches
  bleGamepadConfig.setVid(0x4511);                              // Example VID
  bleGamepadConfig.setPid(0x9023);                              // Example PID
  bleGamepad.begin(&bleGamepadConfig);                          // Start BLE Gamepad
  delay(1000);                                                  // Wait for BLE initialization
}

void loop()
{
  if (bleGamepad.isConnected())
  {
    // Read analog joystick values
    int16_t x_val = analogRead(PIN_X);
    int16_t y_val = analogRead(PIN_Y);

    // Calibrate joystick values
    x_val = map(constrain(x_val - x_offset, x_low, x_high), x_low, x_high, -32767, 32767);
    y_val = map(constrain(y_val - y_offset, y_low, y_high), y_low, y_high, -32767, 32767);

    // Set the gamepad axes
    bleGamepad.setAxes(x_val, y_val);

    // Log joystick values
    Serial.print("Joystick X: ");
    Serial.print(x_val);
    Serial.print(" | Joystick Y: ");
    Serial.println(y_val);

    // Draw joystick representation
    tft.fillCircle(CENTER_X, CENTER_Y, CIRCLE_RADIUS, TFT_BLACK); // Clear previous circle
    tft.drawCircle(CENTER_X, CENTER_Y, CIRCLE_RADIUS, TFT_WHITE);

    // Draw vertical and horizontal lines through the center
    tft.drawLine(CENTER_X - CIRCLE_RADIUS, CENTER_Y, CENTER_X + CIRCLE_RADIUS, CENTER_Y, TFT_WHITE); // Horizontal line
    tft.drawLine(CENTER_X, CENTER_Y - CIRCLE_RADIUS, CENTER_X, CENTER_Y + CIRCLE_RADIUS, TFT_WHITE); // Vertical line

    // Draw inner circle indicating perfect center
    const int16_t INNER_CIRCLE_RADIUS = 8;                              // Radius of the inner circle
    tft.drawCircle(CENTER_X, CENTER_Y, INNER_CIRCLE_RADIUS, TFT_GREEN); // Center dot

    // Calculate the joystick position within the circle
    int16_t x_pos = map(constrain(x_val, -32767, 32767), -32767, 32767, CENTER_X - CIRCLE_RADIUS, CENTER_X + CIRCLE_RADIUS);
    int16_t y_pos = map(constrain(y_val, -32767, 32767), -32767, 32767, CENTER_Y - CIRCLE_RADIUS, CENTER_Y + CIRCLE_RADIUS);

    // Ensure the dot stays within the circle boundary
    int16_t dx = x_pos - CENTER_X;
    int16_t dy = y_pos - CENTER_Y;
    if (dx * dx + dy * dy > CIRCLE_RADIUS * CIRCLE_RADIUS)
    {
      float angle = atan2(dy, dx);
      x_pos = CENTER_X + CIRCLE_RADIUS * cos(angle);
      y_pos = CENTER_Y + CIRCLE_RADIUS * sin(angle);
    }

    // Draw the dot representing the joystick position
    tft.fillCircle(x_pos, y_pos, 7, TFT_RED);

    // Update button states
    for (int i = 0; i < BUTTONS; i++)
    {
      if (digitalRead(button_pins[i]) == LOW)
      {
        bleGamepad.press(button_mapping[i]); // Press button
      }
      else
      {
        bleGamepad.release(button_mapping[i]); // Release button
      }
    }

    bleGamepad.sendReport(); // Send BLE gamepad report
    delay(10);               // Delay for stability
  }
}
