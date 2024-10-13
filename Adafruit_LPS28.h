#ifndef ADAFRUIT_LPS28_H
#define ADAFRUIT_LPS28_H

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

// I2C Address and Register Definitions
#define LPS28_DEFAULT_ADDRESS (0x5D) ///< Default I2C address
#define LPS28_WHOAMI_REG 0x0F        ///< WHO_AM_I register
#define LPS28_WHOAMI_EXPECTED 0xB4   ///< Expected WHO_AM_I value

#define LPS28_THS_P 0x0C             ///< Threshold Pressure (2 bytes)
#define LPS28_CTRL_REG1 0x10         ///< Control Register 1 (ODR and AVG)
#define LPS28_CTRL_REG2 0x11         ///< Control Register 2
#define LPS28_CTRL_REG3 0x12         ///< Control Register 3 (Interrupt config)
#define LPS28_INTERRUPT_CFG 0x0B     ///< Interrupt Configuration Register
#define LPS28_IF_CTRL 0x0E           ///< Interface Control Register

// Output Data Rate (ODR) configuration (Table 19)
typedef enum {
  LPS28_ODR_ONESHOT = 0b0000, ///< One-shot mode
  LPS28_ODR_1_HZ    = 0b0001, ///< 1 Hz
  LPS28_ODR_4_HZ    = 0b0010, ///< 4 Hz
  LPS28_ODR_10_HZ   = 0b0011, ///< 10 Hz
  LPS28_ODR_25_HZ   = 0b0100, ///< 25 Hz
  LPS28_ODR_50_HZ   = 0b0101, ///< 50 Hz
  LPS28_ODR_75_HZ   = 0b0110, ///< 75 Hz
  LPS28_ODR_100_HZ  = 0b0111, ///< 100 Hz
  LPS28_ODR_200_HZ  = 0b1000  ///< 200 Hz
} lps28_odr_t;

// Averaging (AVG) selection (Table 20)
typedef enum {
  LPS28_AVG_4   = 0b000, ///< 4 samples
  LPS28_AVG_8   = 0b001, ///< 8 samples
  LPS28_AVG_16  = 0b010, ///< 16 samples
  LPS28_AVG_32  = 0b011, ///< 32 samples
  LPS28_AVG_64  = 0b100, ///< 64 samples
  LPS28_AVG_128 = 0b101, ///< 128 samples
  LPS28_AVG_512 = 0b111  ///< 512 samples
} lps28_avg_t;

class Adafruit_LPS28 {
public:
  Adafruit_LPS28();

  bool begin(TwoWire *theWire = &Wire, uint8_t i2c_addr = LPS28_DEFAULT_ADDRESS);

  // Threshold Pressure (THS_P) functions
  bool setThresholdPressure(uint16_t threshold);
  uint16_t getThresholdPressure();

  // Data Rate (ODR) functions
  bool setDataRate(lps28_odr_t odr);
  lps28_odr_t getDataRate();

  // Averaging (AVG) functions
  bool setAveraging(lps28_avg_t avg);
  lps28_avg_t getAveraging();

  // CTRL_REG2 functions
  bool rebootMemory();
  bool setFullScaleMode(bool mode);
  bool setLowPassODR9(bool enable);
  bool reset();
  bool triggerOneShot();

  // Interrupt Configuration (CTRL_REG3) functions
  bool setInterruptPin(bool polarity, bool openDrain);

  // Interface Control Register (IF_CTRL) functions
  bool setSDAPullup(bool enable);
  bool setINTPulldown(bool enable);

  // Interrupt Configuration (INTERRUPT_CFG) functions
  bool setAutoReferencePressure(bool enable);
  bool getAutoReferencePressure();
  bool resetAutoReferencePressure();

  bool setAutoZero(bool enable);
  bool getAutoZero();
  bool resetAutoZero();

  bool setPressureInterrupt(bool low, bool high, bool latching);

private:
  Adafruit_I2CDevice *i2c_dev = nullptr; ///< Pointer to I2C device interface
};

#endif // ADAFRUIT_LPS28_H
