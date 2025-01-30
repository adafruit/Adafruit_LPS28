#include <Wire.h>
#include <Adafruit_LPS28.h>

// Instantiate the sensor
Adafruit_LPS28 lps28;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial monitor

  Serial.println("LPS28 One-Shot Mode Test");

  // Initialize the sensor
  if (!lps28.begin()) {
    Serial.println("Failed to find LPS28 sensor!");
    while (1) delay(10);
  }

  Serial.println("LPS28 sensor found!");

  // Set ODR to one-shot mode, and we'll average 512 samples in a row
  // for ultra-stable readings.
  lps28.setDataRate(LPS28_ODR_ONESHOT);
  lps28.setAveraging(LPS28_AVG_512);

  // Enable DRDY interrupt on the interrupt pin
  lps28.setInterruptPin(
    true,  // Polarity: Active high
    false  // Pin mode: Push-pull
  );

  // Enable DRDY interrupt output on INT pin (we could use this with an interrupt)
  lps28.setIntPinOutput(
    true,  // DRDY active
    false, // DRDY pulse not enabled
    false, // INT output not enabled
    false, // FIFO full interrupt not enabled
    false, // FIFO watermark interrupt not enabled
    false  // FIFO overrun interrupt not enabled
  );
}

void loop() {
  // Trigger a one-shot measurement
  lps28.triggerOneShot();

  // Wait (block) until data is ready
  while (!(lps28.getStatus() & LPS28_STATUS_PRESS_READY)) {
    delay(1); // Small delay to allow background tasks to complete
  }

  // Once data is ready, read and print pressure and temperature
  float pressure = lps28.getPressure();
  float temperature = lps28.getTemperature();

  Serial.print("Pressure (hPa): ");
  Serial.println(pressure);

  Serial.print("Temperature (°C): ");
  Serial.println(temperature);
}
