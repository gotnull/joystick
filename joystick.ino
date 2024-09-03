#include <BleGamepad.h>
#include <TFT_eSPI.h>

/*
 * Libraries
 */

// ESP32-BLE-Gamepad 0.5.4
// NimBLE-Arduino    1.4.2
// TFT_eSPI          2.5.43
// SPI               2.0.0
// FS                2.0.0
// SPIFFS            2.0.0

// Analog pins for joystick input
#define PIN_X 18 // ESP32 ADC pin for joystick X-axis
#define PIN_Y 17 // ESP32 ADC pin for joystick Y-axis

// Configuration constants
#define TFT_WIDTH 320             // TFT width
#define TFT_HEIGHT 170            // TFT height
#define BUTTONS 6                 // Number of physical buttons
#define CIRCLE_RADIUS 30          // Radius of the circle representing joystick range
#define CENTER_X (TFT_WIDTH / 2)  // Center X position on the TFT
#define CENTER_Y (TFT_HEIGHT / 2) // Center Y position on the TFT
#define STATUS_Y 26               // Y position for the connection status text

// Calibration values for joystick
int16_t x_offset = 1836 / 2;
int16_t x_low = 700 - x_offset;
int16_t x_high = 3131 - x_offset;

int16_t y_offset = 884 / 2;
int16_t y_low = 140 - y_offset;
int16_t y_high = 2768 - y_offset;

// Calibration control buttons
#define CALIBRATION_START_PIN 1 // Pin for starting calibration
#define CALIBRATION_DONE_PIN 12 // Pin for stopping calibration
#define BUTTON_UP_PIN 3         // Pin for moving up
#define BUTTON_DOWN_PIN 10      // Pin for moving down
#define BUTTON_LEFT_PIN 2       // Pin for moving left
#define BUTTON_RIGHT_PIN 11     // Pin for moving right

// Button configuration
uint8_t button_pins[BUTTONS] = {CALIBRATION_START_PIN, CALIBRATION_DONE_PIN, BUTTON_UP_PIN, BUTTON_DOWN_PIN, BUTTON_LEFT_PIN, BUTTON_RIGHT_PIN}; // Define button pins for ESP32
uint8_t button_mapping[BUTTONS] = {1, 2, 3, 4, 5, 6};                                                                                            // Mapping buttons 1 to 6

// Debounce delay in milliseconds
#define DEBOUNCE_DELAY 300

// TFT Setup
TFT_eSPI tft = TFT_eSPI();

// Initialize BleGamepad
BleGamepad bleGamepad("Gotnull's Gamepad", "gotnull", 100); // Name, Manufacturer, Battery Level
BleGamepadConfiguration bleGamepadConfig;

// Calibration state
bool calibrating = false;
int16_t calibration_x_center = 0;
int16_t calibration_y_center = 0;

// Debounce state
unsigned long last_calibration_start_time = 0;
unsigned long last_calibration_done_time = 0;

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

  delay(2000); // Wait for BLE initialization
}

void loop()
{
  int16_t x_val = analogRead(PIN_X);
  int16_t y_val = analogRead(PIN_Y);

  // Constrain the x and y values
  x_val = map(constrain(x_val - calibration_x_center, x_low, x_high), x_low, x_high, -32767, 32767);
  y_val = map(constrain(y_val - calibration_y_center, y_low, y_high), y_low, y_high, -32767, 32767);

  // Invert the axes for the TFT display
  x_val = -x_val;
  y_val = -y_val;

  unsigned long current_time = millis();

  // CALIBRATION_START_PIN
  if (digitalRead(button_pins[0]) == LOW && (current_time - last_calibration_start_time) > DEBOUNCE_DELAY)
  {
    startCalibration();
    last_calibration_start_time = current_time;
  }

  // Begin calibration
  if (calibrating)
  {
    calibrateJoystick(x_val, y_val);
  }
  else // Operate joysticks
  {
    operateJoystick(x_val, y_val);
  }

  updateButtonStates();    // Update the button states
  bleGamepad.sendReport(); // Send BLE gamepad report

  delay(10); // Delay for stability
}

void startCalibration()
{
  calibrating = true;
  calibration_x_center = analogRead(PIN_X);
  calibration_y_center = analogRead(PIN_Y);
  Serial.println("Calibration started.");
}

void finalizeCalibration()
{
  // Set the offsets to the current center values
  x_offset = calibration_x_center;
  y_offset = calibration_y_center;

  // Use int32_t for the calculations to avoid overflow
  int32_t x_low_32 = calibration_x_center - 32768;
  int32_t x_high_32 = calibration_x_center + 32767;
  int32_t y_low_32 = calibration_y_center - 32768;
  int32_t y_high_32 = calibration_y_center + 32767;

  // Convert back to int16_t if necessary (if staying within valid ranges)
  x_low = (int16_t)constrain(x_low_32, -32768, 32767);
  x_high = (int16_t)constrain(x_high_32, -32768, 32767);
  y_low = (int16_t)constrain(y_low_32, -32768, 32767);
  y_high = (int16_t)constrain(y_high_32, -32768, 32767);

  // Calibration is completed
  calibrating = false;

  // Print the calibration values
  Serial.println("Calibration Values:");
  Serial.print("x_offset: ");
  Serial.println(x_offset);
  Serial.print("y_offset: ");
  Serial.println(y_offset);
  Serial.print("x_low: ");
  Serial.println(x_low);
  Serial.print("x_high: ");
  Serial.println(x_high);
  Serial.print("y_low: ");
  Serial.println(y_low);
  Serial.print("y_high: ");
  Serial.println(y_high);

  // Print calibration completion message
  Serial.println("Calibration completed.");
}

void calibrateJoystick(int16_t x_val, int16_t y_val)
{
  bleGamepad.setAxes(x_val, y_val);

  drawJoystickRepresentation(x_val, y_val);

  // BUTTON_UP_PIN
  if (digitalRead(button_pins[2]) == LOW)
  {
    calibration_y_center -= 10;
  }

  // BUTTON_DOWN_PIN
  if (digitalRead(button_pins[3]) == LOW)
  {
    calibration_y_center += 10;
  }

  // BUTTON_LEFT_PIN
  if (digitalRead(button_pins[4]) == LOW)
  {
    calibration_x_center -= 10;
  }

  // BUTTON_RIGHT_PIN
  if (digitalRead(button_pins[5]) == LOW)
  {
    calibration_x_center += 10;
  }

  // CALIBRATION_DONE_PIN
  if (digitalRead(button_pins[1]) == LOW && (millis() - last_calibration_done_time) > DEBOUNCE_DELAY)
  {
    finalizeCalibration();
    last_calibration_done_time = millis();
  }
}

void operateJoystick(int16_t x_val, int16_t y_val)
{
  bleGamepad.setAxes(x_val, y_val);

  drawJoystickRepresentation(x_val, y_val);
}

void drawJoystickRepresentation(int16_t x_val, int16_t y_val)
{
  // Clear previous circle
  tft.fillCircle(CENTER_X, CENTER_Y, CIRCLE_RADIUS, TFT_BLACK);

  // Draw the joystick circle
  tft.drawCircle(CENTER_X, CENTER_Y, CIRCLE_RADIUS, TFT_WHITE);

  // Draw the circle lines
  tft.drawLine(CENTER_X - CIRCLE_RADIUS, CENTER_Y, CENTER_X + CIRCLE_RADIUS, CENTER_Y, TFT_WHITE); // Horizontal line
  tft.drawLine(CENTER_X, CENTER_Y - CIRCLE_RADIUS, CENTER_X, CENTER_Y + CIRCLE_RADIUS, TFT_WHITE); // Vertical line

  // Draw the center dot
  const int16_t INNER_CIRCLE_RADIUS = 8;                              // Radius of the inner circle
  tft.drawCircle(CENTER_X, CENTER_Y, INNER_CIRCLE_RADIUS, TFT_GREEN); // Center dot

  // Calculate the x and y position of the red circle
  int16_t x_pos = map(constrain(x_val, -32767, 32767), -32767, 32767, CENTER_X - CIRCLE_RADIUS, CENTER_X + CIRCLE_RADIUS);
  int16_t y_pos = map(constrain(y_val, -32767, 32767), -32767, 32767, CENTER_Y - CIRCLE_RADIUS, CENTER_Y + CIRCLE_RADIUS);

  int16_t dx = x_pos - CENTER_X;
  int16_t dy = y_pos - CENTER_Y;
  if (dx * dx + dy * dy > CIRCLE_RADIUS * CIRCLE_RADIUS)
  {
    float angle = atan2(dy, dx);
    x_pos = CENTER_X + CIRCLE_RADIUS * cos(angle);
    y_pos = CENTER_Y + CIRCLE_RADIUS * sin(angle);
  }

  // Draw the red circle
  tft.fillCircle(x_pos, y_pos, 7, TFT_RED);
}

void updateButtonStates()
{
  for (int i = 0; i < BUTTONS; i++)
  {
    if (digitalRead(button_pins[i]) == LOW)
    {
      // Press button
      bleGamepad.press(button_mapping[i]);
    }
    else
    {
      // Release button
      bleGamepad.release(button_mapping[i]);
    }
  }
}
