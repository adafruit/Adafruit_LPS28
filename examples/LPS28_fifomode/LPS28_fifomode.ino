#include <Wire.h>
#include <Adafruit_LPS28.h>

// Instantiate the sensor
Adafruit_LPS28 lps28;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial monitor

  Serial.println("LPS28 FIFO Mode Test");

  // Initialize the sensor
  if (!lps28.begin()) {
    Serial.println("Failed to find LPS28 sensor!");
    while (1) delay(10);
  }

  Serial.println("LPS28 sensor found!");

  // Set ODR to 200 Hz for fast sampling and averaging to 512 samples
  lps28.setDataRate(LPS28_ODR_200_HZ);
  lps28.setAveraging(LPS28_AVG_512);

  // Enable FIFO
  lps28.setFIFOmode(false, LPS28_FIFO_CONTINUOUS);

  // Set FIFO watermark level to 10 samples
  lps28.setFIFOWatermark(10);

  // Enable FIFO Watermark Interrupt in CTRL_REG4, could use IRQ pin!
  lps28.setIntPinOutput(
    false, // DRDY not enabled
    false, // DRDY pulse not enabled
    false, // INT output not enabled
    false, // FIFO full interrupt not enabled
    true,  // FIFO watermark interrupt enabled
    false  // FIFO overrun interrupt not enabled
  );
}

void loop() {
  // Check if FIFO watermark is reached
  if (lps28.getFIFOstatus() & LPS28_FIFO_STATUS_WTM_IA) {
    uint8_t unreadSamples = lps28.getFIFOunreadSamples();

    Serial.print("FIFO unread samples: ");
    Serial.println(unreadSamples);

    for (uint8_t i = 0; i < unreadSamples; i++) {
      float pressure = lps28.getFIFOpressure();
      Serial.print("FIFO Pressure (hPa): ");
      Serial.println(pressure);
    }
  }

  delay(10); // Short delay to avoid excessive polling
}
