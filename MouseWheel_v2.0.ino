#include <SPI.h>            // SPI library for accessing SD card
#include <SD.h>             // SD card library
#include "esp_sleep.h"      // ESP32 sleep modes
#include "driver/rtc_io.h"  // RTC GPIO control
#include <Wire.h>           // I2C library for accessing RTC
#include <time.h>           // Library for time data format
#include "RTClib.h"

#define LED_PIN LED_BUILTIN     // Define your LED pin, adjust if needed (built-in LED is usually GPIO 2)
#define uS_TO_S_FACTOR 1000000  // Conversion factor for seconds to microseconds
#define TIME_TO_SLEEP 5         // Time in seconds for sleep

// IO definitions
#define EXT_SD_CS 10           // Chip-Select of SD card slot on RTC shield
#define SENSOR_IO GPIO_NUM_18  // A0 corresponds to GPIO 18
#define RTC_INT 12             // RTC interrupt pin
#define VBATPIN A7             // IO to measure battery voltage

RTC_PCF8523 rtc;

bool waitForLow = true;  // Start by waiting for the input to go low

void setup() {
  // Define IOs
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  pinMode(RTC_INT, INPUT_PULLUP);
  pinMode(EXT_SD_CS, OUTPUT);

  Serial.begin(115200);
  delay(1000);  // Allow time for Serial monitor to connect

  // Configure the RTC GPIO pull-up to maintain it during sleep
  rtc_gpio_init(SENSOR_IO);                                     // Initialize the RTC GPIO
  rtc_gpio_set_direction(SENSOR_IO, RTC_GPIO_MODE_INPUT_ONLY);  // Set as input
  rtc_gpio_pullup_en(SENSOR_IO);                                // Enable the pull-up resistor
  rtc_gpio_pulldown_dis(SENSOR_IO);                             // Disable the pull-down resistor

  // Check the initial state of the sensor pin
  if (digitalRead(18) == LOW) {
    // If the sensor starts LOW, configure to wake on HIGH
    waitForLow = false;      // We're waiting for the signal to go HIGH
    configureWakeUp(false);  // Set wake-up for HIGH
    Serial.println("Initial state LOW, configured to wake up on HIGH.");
  } else {
    // If the sensor starts HIGH, configure to wake on LOW
    waitForLow = true;      // We're waiting for the signal to go LOW
    configureWakeUp(true);  // Set wake-up for LOW
    Serial.println("Initial state HIGH, configured to wake up on LOW.");
  }

  // Initialize RTC and SD card (omitted for brevity)
  // initializeRTCandSD();

  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Indicate that the device is entering light sleep by turning off the LED
  Serial.println("Entering light sleep...");
  delay(100);                  // Give time for the serial message to be sent
  digitalWrite(LED_PIN, LOW);  // Turn off LED before sleep

  // Enter light sleep mode
  esp_light_sleep_start();  // Go to sleep and wait for wake-up condition

  // Wake up here
  delay(100);  // Small delay to allow peripherals to reinitialize after waking

  // Indicate wake-up with LED or GPIO
  digitalWrite(LED_PIN, HIGH);  // Turn on LED to indicate wake-up

  if (waitForLow) {
    Serial.println("Woke up! Input went low.");
    performLogging();  // Your logging function goes here

    waitForLow = false;      // Now wait for the input to go high
    configureWakeUp(false);  // Set wake-up for HIGH
  } else {
    Serial.println("Woke up! Input went high, going back to detect low.");
    waitForLow = true;      // Now wait for the input to go low
    configureWakeUp(true);  // Set wake-up for LOW
  }

  // Flash LED for 200ms to signal activity
  digitalWrite(LED_PIN, HIGH);
  delay(200);
  digitalWrite(LED_PIN, LOW);
}


void configureWakeUp(bool detectLow) {
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);  // Disable the previous wakeup source

  if (detectLow) {
    // Configure ext0 wakeup to trigger on falling edge (LOW)
    esp_sleep_enable_ext0_wakeup(SENSOR_IO, 0);
    Serial.println("Configured to wake up on input going LOW.");
  } else {
    // Configure ext0 wakeup to trigger on rising edge (HIGH)
    esp_sleep_enable_ext0_wakeup(SENSOR_IO, 1);
    Serial.println("Configured to wake up on input going HIGH.");
  }
}

void performLogging() {
}

void initializeRTCandSD() {
  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC");
    while (1) delay(10);
  }

  if (!rtc.initialized() || rtc.lostPower()) {
    Serial.println("RTC is NOT initialized, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
  rtc.start();
  printNow();

  // slow down SPI due to stacked boards
  if (SD.begin(EXT_SD_CS, SPI, 1000000)) {
    Serial.println("SD card online.");
  } else {
    Serial.println("SD card not detected.");
  }
}

void printNow() {
  DateTime now = rtc.now();
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}
