#ifndef ADAFRUIT_LPS28_H
#define ADAFRUIT_LPS28_H

#include <Arduino.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_BusIO_Register.h>

// I2C Address and Register Definitions
#define LPS28_DEFAULT_ADDRESS (0x5C) ///< Default I2C address
#define LPS28_WHOAMI_REG 0x0F        ///< WHO_AM_I register
#define LPS28_WHOAMI_EXPECTED 0xB4   ///< Expected WHO_AM_I value

#define LPS28_THS_P 0x0C             ///< Threshold Pressure (2 bytes)
#define LPS28_CTRL_REG1 0x10         ///< Control Register 1 (ODR and AVG)
#define LPS28_CTRL_REG2 0x11         ///< Control Register 2
#define LPS28_CTRL_REG3 0x12         ///< Control Register 3 (Interrupt config)
#define LPS28_CTRL_REG4 0x13 ///< Control Register 4
#define LPS28_INTERRUPT_CFG 0x0B     ///< Interrupt Configuration Register
#define LPS28_IF_CTRL 0x0E           ///< Interface Control Register
#define LPS28_FIFO_CTRL 0x14 ///< FIFO Control Register
#define LPS28_FIFO_WTM 0x15 ///< FIFO Watermark Register
#define LPS28_REF_P 0x16 ///< Reference Pressure Register (2 bytes)
#define LPS28_RPDS 0x18 ///< RPDS Register (Pressure Offset, 2 bytes)
#define LPS28_INT_SOURCE 0x24 ///< INT_SOURCE Register
#define LPS28_FIFO_STATUS1 0x25 ///< FIFO_STATUS1 Register
#define LPS28_FIFO_STATUS2 0x26 ///< FIFO_STATUS2 Register
#define LPS28_STATUS 0x27 ///< STATUS Register
#define LPS28_PRESS_OUT 0x28 ///< PRESS_OUT Register (3 bytes)
#define LPS28_TEMP_OUT 0x2B ///< TEMP_OUT Register (2 bytes)
#define LPS28_FIFO_DATA_OUT_PRESS_XL 0x78 ///< FIFO_DATA_OUT_PRESS_XL Register (3 bytes)



// STATUS flags
#define LPS28_STATUS_TEMP_OVERRUN 0x20 ///< Temperature data overrun (bit 5)
#define LPS28_STATUS_PRESS_OVERRUN 0x10 ///< Pressure data overrun (bit 4)
#define LPS28_STATUS_TEMP_READY   0x02 ///< Temperature data available (bit 1)
#define LPS28_STATUS_PRESS_READY  0x01 ///< Pressure data available (bit 0)

// FIFO_STATUS2 flags
#define LPS28_FIFO_STATUS_WTM_IA  0x80 ///< FIFO Watermark interrupt active (bit 7)
#define LPS28_FIFO_STATUS_OVR_IA  0x40 ///< FIFO Overrun interrupt active (bit 6)
#define LPS28_FIFO_STATUS_FULL_IA 0x20 ///< FIFO Full interrupt active (bit 5)

// INT_SOURCE flags
#define LPS28_INT_SOURCE_IA      0x20  ///< Interrupt Active flag (bit 5)
#define LPS28_INT_SOURCE_PL      0x10  ///< Low pressure event flag (bit 4)
#define LPS28_INT_SOURCE_PH      0x08  ///< High pressure event flag (bit 3)
#define LPS28_INT_SOURCE_BOOT_ON 0x80  ///< Boot status flag (bit 7)


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


// FIFO Mode Selection based on TRIG_MODE and F_MODE bits
typedef enum {
  LPS28_FIFO_BYPASS                   = 0b00, ///< Bypass mode (F_MODE[1:0] = 00)
  LPS28_FIFO_FIFO                     = 0b01, ///< FIFO mode (TRIG = 0, F_MODE[1:0] = 01)
  LPS28_FIFO_CONTINUOUS               = 0b10, ///< Continuous mode (TRIG = 0, F_MODE[1:0] = 1x)
  LPS28_FIFO_BYPASS_TO_FIFO           = 0b101, ///< Bypass-to-FIFO mode (TRIG = 1, F_MODE[1:0] = 01)
  LPS28_FIFO_BYPASS_TO_CONTINUOUS     = 0b110, ///< Bypass-to-Continuous mode (TRIG = 1, F_MODE[1:0] = 10)
  LPS28_FIFO_CONTINUOUS_TO_FIFO       = 0b111  ///< Continuous-to-FIFO mode (TRIG = 1, F_MODE[1:0] = 11)
} lps28_fifo_mode_t;



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
  bool getFullScaleMode();
  bool setLowPassODR9(bool enable);
  bool reset();
  bool triggerOneShot();

  // Interrupt Configuration (CTRL_REG3) functions
  bool setInterruptPin(bool polarity, bool openDrain);
  bool setIntPinOutput(bool drdy, bool drdy_pulse, bool int_enable,
                       bool fifo_full, bool fifo_watermark, bool fifo_overrun);

  // Interface Control Register (IF_CTRL) functions
  bool setSDAPullup(bool enable);
  bool setINTPulldown(bool enable);

  // Interrupt Configuration (INTERRUPT_CFG) functions
  bool setAutoReferencePressure(bool enable);
  bool getAutoReferencePressure();
  bool resetAutoReferencePressure();

  int16_t getReferencePressure();
  bool setPressureOffset(int16_t offset);
  int16_t getPressureOffset();

  bool setAutoZero(bool enable);
  bool getAutoZero();
  bool resetAutoZero();

  bool setPressureInterrupt(bool low, bool high, bool latching);
  uint8_t getIntSource();

  // FIFO
  bool setFIFOWatermark(uint8_t wtm);
  uint8_t getFIFOWatermark(); 
  uint8_t getFIFOunreadSamples();
  uint8_t getFIFOstatus();
  bool setFIFOmode(bool stop_on_watermark, lps28_fifo_mode_t mode);

  uint8_t getStatus();
  float getPressure();
  float getTemperature();
  float getFIFOpressure();

private:
  Adafruit_I2CDevice *i2c_dev = nullptr; ///< Pointer to I2C device interface
};

#endif // ADAFRUIT_LPS28_H
