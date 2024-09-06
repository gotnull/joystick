// Testing: https://hcidesign.com/gamepad/
// arduino-cli compile --fqbn esp32:esp32:lilygo_t_display_s3
// arduino-cli upload -p /dev/cu.usbmodem13301 --fqbn esp32:esp32:lilygo_t_display_s3
// arduino-cli monitor -p /dev/cu.usbmodem13301 --config 115200

#include <EEPROM.h>
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

// Calibration values for joystick
// int16_t x_offset = 0;
// int16_t x_low = 0;
// int16_t x_high = 0;
// int16_t y_offset = 0;
// int16_t y_low = 0;
// int16_t y_high = 0;

int16_t x_offset = 3229;
int16_t y_offset = 4287;
int16_t x_low = -1677;
int16_t x_high = 754;
int16_t y_low = -2763;
int16_t y_high = -135;

// Analog pins for joystick input
#define PIN_X 17 // ESP32 ADC pin for joystick X-axis
#define PIN_Y 18 // ESP32 ADC pin for joystick Y-axis

// TFT constants
#define TFT_WIDTH 320  // TFT width
#define TFT_HEIGHT 170 // TFT height

// Joystick constants
#define BUTTONS 6                 // Number of physical buttons
#define CIRCLE_RADIUS 30          // Radius of the circle representing joystick range
#define CENTER_X (TFT_WIDTH / 2)  // Center X position on the TFT
#define CENTER_Y (TFT_HEIGHT / 2) // Center Y position on the TFT
#define STATUS_Y 26               // Y position for the connection status text

// Calibration control buttons
#define CALIBRATION_START_PIN 1 // Pin for starting calibration
#define CALIBRATION_DONE_PIN 12 // Pin for stopping calibration
#define BUTTON_UP_PIN 3         // Pin for moving up
#define BUTTON_DOWN_PIN 10      // Pin for moving down
#define BUTTON_LEFT_PIN 2       // Pin for moving left
#define BUTTON_RIGHT_PIN 11     // Pin for moving right

// Button configuration
uint8_t button_pins[BUTTONS] = {CALIBRATION_START_PIN, CALIBRATION_DONE_PIN, BUTTON_UP_PIN, BUTTON_DOWN_PIN, BUTTON_LEFT_PIN, BUTTON_RIGHT_PIN};
uint8_t button_mapping[BUTTONS] = {1, 2, 3, 4, 5, 6};

// Debounce delay in milliseconds
#define DEBOUNCE_DELAY 300

// TFT Setup
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft); // Add sprite for smooth rendering

// Initialize BleGamepad
BleGamepad bleGamepad("Gotnull's Gamepad", "gotnull", 100);
BleGamepadConfiguration bleGamepadConfig;

// Calibration state
bool calibrating = false;
int16_t calibration_x_center = 0;
int16_t calibration_y_center = 0;

// Debounce state
unsigned long last_calibration_start_time = 0;
unsigned long last_calibration_done_time = 0;

// EEPROM addresses for calibration values
#define EEPROM_X_OFFSET_ADDR 0
#define EEPROM_X_LOW_ADDR (EEPROM_X_OFFSET_ADDR + sizeof(int16_t))
#define EEPROM_X_HIGH_ADDR (EEPROM_X_LOW_ADDR + sizeof(int16_t))
#define EEPROM_Y_OFFSET_ADDR (EEPROM_X_HIGH_ADDR + sizeof(int16_t))
#define EEPROM_Y_LOW_ADDR (EEPROM_Y_OFFSET_ADDR + sizeof(int16_t))
#define EEPROM_Y_HIGH_ADDR (EEPROM_Y_LOW_ADDR + sizeof(int16_t))

void saveCalibrationValues()
{
  EEPROM.put(EEPROM_X_OFFSET_ADDR, x_offset);
  EEPROM.put(EEPROM_X_LOW_ADDR, x_low);
  EEPROM.put(EEPROM_X_HIGH_ADDR, x_high);
  EEPROM.put(EEPROM_Y_OFFSET_ADDR, y_offset);
  EEPROM.put(EEPROM_Y_LOW_ADDR, y_low);
  EEPROM.put(EEPROM_Y_HIGH_ADDR, y_high);
  EEPROM.commit();
}

void loadCalibrationValues()
{
  EEPROM.get(EEPROM_X_OFFSET_ADDR, x_offset);
  EEPROM.get(EEPROM_X_LOW_ADDR, x_low);
  EEPROM.get(EEPROM_X_HIGH_ADDR, x_high);
  EEPROM.get(EEPROM_Y_OFFSET_ADDR, y_offset);
  EEPROM.get(EEPROM_Y_LOW_ADDR, y_low);
  EEPROM.get(EEPROM_Y_HIGH_ADDR, y_high);
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE Gamepad...");

  EEPROM.begin(512);
  loadCalibrationValues();

  tft.init();
  tft.setRotation(1);
  sprite.createSprite(TFT_WIDTH, TFT_HEIGHT); // Create sprite to cover the whole screen

  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);             // Middle Center datum for text
  sprite.setTextColor(TFT_WHITE, TFT_BLACK); // Text color white, background black
  sprite.drawString("Waiting...", tft.width() / 2, STATUS_Y, 4);
  sprite.pushSprite(0, 0); // Push sprite to show text

  pinMode(PIN_X, INPUT);
  pinMode(PIN_Y, INPUT);

  for (int i = 0; i < BUTTONS; i++)
  {
    pinMode(button_pins[i], INPUT_PULLUP);
  }

  bleGamepadConfig.setAutoReport(false);
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_JOYSTICK);
  bleGamepadConfig.setButtonCount(BUTTONS);
  bleGamepadConfig.setHatSwitchCount(0);
  bleGamepadConfig.setVid(0x4511);
  bleGamepadConfig.setPid(0x9023);
  bleGamepad.begin(&bleGamepadConfig);

  printCalibration();

  delay(2000);
}

void loop()
{
  int16_t x_val = analogRead(PIN_X);
  int16_t y_val = analogRead(PIN_Y);

  // Check BLE connection status
  sprite.fillSprite(TFT_BLACK); // Clear sprite
  if (bleGamepad.isConnected())
  {
    sprite.drawString("Connected", CENTER_X, STATUS_Y, 2); // Draw connected status
  }
  else
  {
    sprite.drawString("Not Connected", CENTER_X, STATUS_Y, 2); // Draw not connected status
  }

  // Calibration logic
  if (calibrating)
  {
    x_val = map(constrain(x_val - calibration_x_center, x_low, x_high), x_low, x_high, -32767, 32767);
    y_val = map(constrain(y_val - calibration_y_center, y_low, y_high), y_low, y_high, -32767, 32767);
  }
  else
  {
    x_val = map(constrain(x_val - x_offset, x_low, x_high), x_low, x_high, -32767, 32767);
    y_val = map(constrain(y_val - y_offset, y_low, y_high), y_low, y_high, -32767, 32767);
  }

  x_val = -x_val;
  y_val = -y_val;

  unsigned long current_time = millis();

  if (digitalRead(button_pins[0]) == LOW && (current_time - last_calibration_start_time) > DEBOUNCE_DELAY)
  {
    startCalibration();
    last_calibration_start_time = current_time;
  }

  if (calibrating)
  {
    calibrateJoystick(x_val, y_val);
  }
  else
  {
    operateJoystick(x_val, y_val);
  }

  updateButtonStates();
  bleGamepad.sendReport();

  drawJoystickRepresentation(x_val, y_val); // Draw joystick after text

  sprite.pushSprite(0, 0); // Push sprite after drawing everything for smooth rendering
  delay(10);
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
  x_offset = calibration_x_center;
  y_offset = calibration_y_center;

  printCalibration();
  saveCalibrationValues();
  calibrating = false;

  Serial.println("Calibration completed.");
}

void printCalibration()
{
  Serial.println("Calibration Values:");

  Serial.print("int16_t x_offset = ");
  Serial.print(x_offset);
  Serial.println(";");

  Serial.print("int16_t y_offset = ");
  Serial.print(y_offset);
  Serial.println(";");

  Serial.print("int16_t x_low = ");
  Serial.print(x_low);
  Serial.println(";");

  Serial.print("int16_t x_high = ");
  Serial.print(x_high);
  Serial.println(";");

  Serial.print("int16_t y_low = ");
  Serial.print(y_low);
  Serial.println(";");

  Serial.print("int16_t y_high = ");
  Serial.print(y_high);
  Serial.println(";");
}

void calibrateJoystick(int16_t x_val, int16_t y_val)
{
  bleGamepad.setAxes(x_val, y_val);
  drawJoystickRepresentation(x_val, y_val);

  if (digitalRead(button_pins[2]) == LOW)
  {
    calibration_y_center -= 10;
  }

  if (digitalRead(button_pins[3]) == LOW)
  {
    calibration_y_center += 10;
  }

  if (digitalRead(button_pins[4]) == LOW)
  {
    calibration_x_center -= 10;
  }

  if (digitalRead(button_pins[5]) == LOW)
  {
    calibration_x_center += 10;
  }

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
  sprite.drawCircle(CENTER_X, CENTER_Y, CIRCLE_RADIUS, TFT_WHITE);
  sprite.drawLine(CENTER_X - CIRCLE_RADIUS, CENTER_Y, CENTER_X + CIRCLE_RADIUS, CENTER_Y, TFT_WHITE);
  sprite.drawLine(CENTER_X, CENTER_Y - CIRCLE_RADIUS, CENTER_X, CENTER_Y + CIRCLE_RADIUS, TFT_WHITE);

  const int16_t INNER_CIRCLE_RADIUS = 8;
  sprite.drawCircle(CENTER_X, CENTER_Y, INNER_CIRCLE_RADIUS, TFT_GREEN);

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

  sprite.fillCircle(x_pos, y_pos, 7, TFT_RED);
}

void updateButtonStates()
{
  for (int i = 0; i < BUTTONS; i++)
  {
    if (digitalRead(button_pins[i]) == LOW)
    {
      bleGamepad.press(button_mapping[i]);
    }
    else
    {
      bleGamepad.release(button_mapping[i]);
    }
  }
}
