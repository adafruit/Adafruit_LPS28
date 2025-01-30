/***************************************************************************
  This is a library for the LPS28 Pressure & Temperature Sensor

  Designed specifically to work with the Adafruit LPS28 Breakout
  ----> http://www.adafruit.com/products/XXXX

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Adafruit_LPS28.h>

Adafruit_LPS28 lps;
Adafruit_Sensor *lps_temp, *lps_pressure;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LPS28 test!");

  // Try to initialize!
  if (!lps.begin()) {
    Serial.println("Failed to find LPS28 chip");
    while (1) { delay(10); }
  }
  Serial.println("LPS28 Found!");

  // Set range to 4060 hPa
  lps.setFullScaleMode(true);
  
  // Get pointers to the sensors for the unified sensor API
  lps_temp = lps.getTemperatureSensor();
  lps_temp->printSensorDetails();

  lps_pressure = lps.getPressureSensor();
  lps_pressure->printSensorDetails();
}

void loop() {
  sensors_event_t pressure;
  sensors_event_t temp;
  
  // Get readings from both sensors
  lps_pressure->getEvent(&pressure);
  lps_temp->getEvent(&temp);
  
  // Pretty print for standard output
  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  Serial.print("Pressure: ");
  Serial.print(pressure.pressure);
  Serial.println(" hPa");
  
  Serial.println();
  delay(100);

  /* Uncomment for serial plotter format
  // Serial plotter friendly format
  Serial.print(temp.temperature);
  Serial.print(",");
  Serial.println(pressure.pressure);
  delay(10);
  */
}
