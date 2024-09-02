// Compile: arduino-cli compile --fqbn esp32:esp32:ttgo-t1 .
// Upload: arduino-cli upload -p /dev/cu.usbserial-0206671E --fqbn esp32:esp32:ttgo-t1
// Monitor: arduino-cli monitor -p /dev/cu.usbserial-0206671E --config 115200

// Tester: https://hardwaretester.com/gamepad

#include <Arduino.h>
#include <BleGamepad.h>
#include <TFT_eSPI.h>

// Define joystick pins for ESP32
#define PIN_X 25 // Joystick X pin (GPIO34)
#define PIN_Y 26 // Joystick Y pin (GPIO35)

#define BUTTONS 6                                     // Number of buttons
uint8_t button_pins[BUTTONS] = {11, 10, 1, 0, 7, 6};  // Button pins for ESP32
uint8_t button_mapping[BUTTONS] = {1, 2, 3, 4, 7, 8}; // Button mapping for BLE gamepad

// Calibration values
const int16_t x_offset = 2300 / 2;
const int16_t x_low = 800 - x_offset;
const int16_t x_high = 3731 - x_offset;

const int16_t y_offset = 1750 / 2;
const int16_t y_low = 520 - y_offset;
const int16_t y_high = 3338 - y_offset;

// TFT Setup
TFT_eSPI tft = TFT_eSPI();

// Initialize BLE gamepad and configuration
BleGamepad bleGamepad("ESP32 Gamepad", "Maker101 Home");
BleGamepadConfiguration bleGamepadConfig;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE Gamepad...");

  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.drawString("Waiting...", tft.width() / 2, 25, 4);

  // Initialize button pins
  // for (int i = 0; i < BUTTONS; i++)
  // {
  //   pinMode(button_pins[i], INPUT_PULLUP);
  // }

  // Configure BLE Gamepad
  bleGamepadConfig.setAutoReport(false);                       // Set to manual report mode
  bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD); // Gamepad type
  bleGamepad.begin(&bleGamepadConfig);                         // Start BLE Gamepad

  delay(1000); // Wait for BLE initialization
  Serial.println("BLE Gamepad started.");
}

void loop()
{
  if (bleGamepad.isConnected())
  {
    // Read joystick values (0...4095, 3.3v)
    int16_t x_val = analogRead(PIN_X);
    int16_t y_val = analogRead(PIN_Y);

    // Apply calibration to joystick values
    x_val = map(constrain(x_val - x_offset, x_low, x_high), x_low, x_high, -32767, 32767);
    y_val = map(constrain(y_val - y_offset, y_low, y_high), y_low, y_high, -32767, 32767);

    // Log joystick values
    Serial.print("Joystick X: ");
    Serial.print(x_val);
    Serial.print(" | Joystick Y: ");
    Serial.println(y_val);

    // Set joystick positions
    bleGamepad.setLeftThumb(x_val, y_val);

    // Handle button states
    // for (int i = 0; i < BUTTONS; i++)
    // {
    //   bool buttonState = !digitalRead(button_pins[i]);
    //   if (buttonState)
    //   {
    //     bleGamepad.press(button_mapping[i]); // Press button
    //     Serial.print("Button ");
    //     Serial.print(button_mapping[i]);
    //     Serial.println(" pressed.");
    //   }
    //   else
    //   {
    //     bleGamepad.release(button_mapping[i]); // Release button
    //     Serial.print("Button ");
    //     Serial.print(button_mapping[i]);
    //     Serial.println(" released.");
    //   }
    // }

    // Send manual report
    bleGamepad.sendReport();
    // Serial.println("BLE report sent.");
  }
  else
  {
    Serial.println("BLE Gamepad not connected.");
  }

  delay(100); // Increased delay to reduce serial output rate
}
