/**
 * @file Adafruit_LPS28.cpp
 * @brief Driver for the LPS28 Pressure Sensor
 * @author Adafruit Industries
 * @copyright BSD License
 */

#include "Adafruit_LPS28.h"

/**
 * @brief Construct a new Adafruit LPS28 object
 */
Adafruit_LPS28::Adafruit_LPS28() {}

/**
 * @brief Initializes the sensor with given parameters, tried reading
 * and veriying the WHOAMI register
 * 
 * @param theWire The Wire object to be used for I2C communication
 * @param i2c_addr The I2C address of the sensor
 * @return true If sensor initialization was successful
 * @return false If sensor initialization failed
 */
bool Adafruit_LPS28::begin(TwoWire *theWire, uint8_t i2c_addr) {
  i2c_dev = new Adafruit_I2CDevice(i2c_addr, theWire);
  if (!i2c_dev->begin()) {
    return false;
  }

  Adafruit_BusIO_Register whoami_reg(i2c_dev, LPS28_WHOAMI_REG, 1);
  return (whoami_reg.read() == LPS28_WHOAMI_EXPECTED);
}

/**
 * @brief Sets the pressure threshold for interrupt generation
 * 
 * @param threshold The pressure threshold value to set
 * @return true If threshold was set successfully
 * @return false If threshold setting failed
 */
bool Adafruit_LPS28::setThresholdPressure(uint16_t threshold) {
  Adafruit_BusIO_Register ths_p(i2c_dev, LPS28_THS_P, 2, LSBFIRST);
  return ths_p.write(threshold);
}

/**
 * @brief Gets the current pressure threshold value for the IRQ
 * 
 * @return uint16_t The current pressure threshold value
 */
uint16_t Adafruit_LPS28::getThresholdPressure() {
  Adafruit_BusIO_Register ths_p(i2c_dev, LPS28_THS_P, 2, LSBFIRST);
  return ths_p.read();
}

/**
 * @brief Sets the data output rate of the sensor (ODR)
 * 
 * @param odr The desired output data rate from lps28_odr_t enum
 * @return true If data rate was set successfully
 * @return false If data rate setting failed
 */
bool Adafruit_LPS28::setDataRate(lps28_odr_t odr) {
  Adafruit_BusIO_Register ctrl_reg1(i2c_dev, LPS28_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits odr_bits(&ctrl_reg1, 4, 3);
  return odr_bits.write(odr);
}

/**
 * @brief Gets the current data output rate setting (ODR)
 * 
 * @return lps28_odr_t The current output data rate setting
 */
lps28_odr_t Adafruit_LPS28::getDataRate() {
  Adafruit_BusIO_Register ctrl_reg1(i2c_dev, LPS28_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits odr_bits(&ctrl_reg1, 4, 3);
  return (lps28_odr_t)odr_bits.read();
}


/**
 * @brief Sets the number of pressure and temperature samples to average
 * 
 * @param avg The averaging setting from lps28_avg_t enum
 * @return true If averaging setting was set successfully
 * @return false If averaging setting failed
 */
bool Adafruit_LPS28::setAveraging(lps28_avg_t avg) {
  Adafruit_BusIO_Register ctrl_reg1(i2c_dev, LPS28_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits avg_bits(&ctrl_reg1, 3, 0);
  return avg_bits.write(avg);
}

/**
 * @brief Gets the current averaging setting
 * 
 * @return lps28_avg_t The current averaging setting
 */
lps28_avg_t Adafruit_LPS28::getAveraging() {
  Adafruit_BusIO_Register ctrl_reg1(i2c_dev, LPS28_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits avg_bits(&ctrl_reg1, 3, 0);
  return (lps28_avg_t)avg_bits.read();
}

/**
 * @brief Reboots the memory content of the sensor
 * 
 * @return true If memory reboot was successful
 * @return false If memory reboot failed
 */
bool Adafruit_LPS28::rebootMemory() {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits boot_bit(&ctrl_reg2, 1, 7);
  return boot_bit.write(true);
}

/**
 * @brief Sets the full scale mode of the sensor
 * 
 * @param mode true for FS_MODE=1 (1/2048 hPa/LSB), false for FS_MODE=0 (1/4096 hPa/LSB)
 * @return true If full scale mode was set successfully
 * @return false If full scale mode setting failed
 */
bool Adafruit_LPS28::setFullScaleMode(bool mode) {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits fs_mode_bit(&ctrl_reg2, 1, 6);
  return fs_mode_bit.write(mode);
}

/**
 * @brief Gets the current full scale mode setting
 * 
 * @return true FS_MODE=1 (1/2048 hPa/LSB)
 * @return false FS_MODE=0 (1/4096 hPa/LSB)
 */
bool Adafruit_LPS28::getFullScaleMode() {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits fs_mode_bit(&ctrl_reg2, 1, 6);
  return fs_mode_bit.read();
}

/**
 * @brief Enables or disables the low-pass filter at ODR/9
 * 
 * @param enable true to enable the low-pass filter, false to disable
 * @return true If setting was applied successfully
 * @return false If setting failed
 */
bool Adafruit_LPS28::setLowPassODR9(bool enable) {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits lpfp_cfg_bit(&ctrl_reg2, 1, 5);
  return lpfp_cfg_bit.write(enable);
}


/**
 * @brief Performs a software reset of the sensor
 * 
 * @return true If reset was successful
 * @return false If reset failed
 */
bool Adafruit_LPS28::reset() {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits swreset_bit(&ctrl_reg2, 1, 2);
  return swreset_bit.write(true);
}

/**
 * @brief Triggers a one-shot measurement
 * 
 * @return true If one-shot measurement was triggered successfully
 * @return false If trigger failed
 */
bool Adafruit_LPS28::triggerOneShot() {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits oneshot_bit(&ctrl_reg2, 1, 0);
  return oneshot_bit.write(true);
}

/**
 * @brief Configures the interrupt pin settings
 * 
 * @param polarity true for active-high, false for active-low
 * @param openDrain true for open-drain output, false for push-pull
 * @return true If interrupt pin configuration was successful
 * @return false If configuration failed
 */
bool Adafruit_LPS28::setInterruptPin(bool polarity, bool openDrain) {
  Adafruit_BusIO_Register ctrl_reg3(i2c_dev, LPS28_CTRL_REG3, 1);
  Adafruit_BusIO_RegisterBits int_h_l_bit(&ctrl_reg3, 1, 3);
  Adafruit_BusIO_RegisterBits pp_od_bit(&ctrl_reg3, 1, 1);
  return int_h_l_bit.write(polarity) && pp_od_bit.write(openDrain);
}

/**
 * @brief Enables or disables the SDA line internal pull-up resistor
 * 
 * @param enable true to enable pull-up, false to disable
 * @return true If pull-up setting was changed successfully
 * @return false If setting change failed
 */
bool Adafruit_LPS28::setSDAPullup(bool enable) {
  Adafruit_BusIO_Register if_ctrl(i2c_dev, LPS28_IF_CTRL, 1);
  Adafruit_BusIO_RegisterBits sda_pu_bit(&if_ctrl, 1, 4);
  return sda_pu_bit.write(enable);
}

/**
 * @brief Enables or disables the INT pin internal pull-down resistor
 * 
 * @param enable true to enable pull-down, false to disable
 * @return true If pull-down setting was changed successfully
 * @return false If setting change failed
 */
bool Adafruit_LPS28::setINTPulldown(bool enable) {
  Adafruit_BusIO_Register if_ctrl(i2c_dev, LPS28_IF_CTRL, 1);
  Adafruit_BusIO_RegisterBits int_pd_dis_bit(&if_ctrl, 1, 2);
  return int_pd_dis_bit.write(!enable);
}

/**
 * @brief Sets up automatic reference pressure mode
 * 
 * @param enable true to enable auto reference pressure, false to disable
 * @return true If auto reference pressure setting was changed successfully
 * @return false If setting change failed
 */
bool Adafruit_LPS28::setAutoReferencePressure(bool enable) {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits autozero_bit(&interrupt_cfg, 1, 7);
  return autozero_bit.write(enable);
}


/**
 * @brief Gets the current auto reference pressure setting
 * 
 * @return true If auto reference pressure is enabled
 * @return false If auto reference pressure is disabled
 */
bool Adafruit_LPS28::getAutoReferencePressure() {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits autozero_bit(&interrupt_cfg, 1, 7);
  return autozero_bit.read();
}

/**
 * @brief Resets the auto reference pressure setting
 * 
 * @return true If reset was successful
 * @return false If reset failed
 */
bool Adafruit_LPS28::resetAutoReferencePressure() {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits reset_arp_bit(&interrupt_cfg, 1, 6);
  return reset_arp_bit.write(true);
}

/**
 * @brief Enables or disables the auto-zero function
 * 
 * @param enable true to enable auto-zero, false to disable
 * @return true If auto-zero setting was changed successfully
 * @return false If setting change failed
 */
bool Adafruit_LPS28::setAutoZero(bool enable) {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits autozero_bit(&interrupt_cfg, 1, 5);
  return autozero_bit.write(enable);
}

/**
 * @brief Gets the current auto-zero setting
 * 
 * @return true If auto-zero is enabled
 * @return false If auto-zero is disabled
 */
bool Adafruit_LPS28::getAutoZero() {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits autozero_bit(&interrupt_cfg, 1, 5);
  return autozero_bit.read();
}

/**
 * @brief Resets the auto-zero function
 * 
 * @return true If reset was successful
 * @return false If reset failed
 */
bool Adafruit_LPS28::resetAutoZero() {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits reset_az_bit(&interrupt_cfg, 1, 4);
  return reset_az_bit.write(true);
}

/**
 * @brief Configures pressure interrupt settings
 * 
 * @param low Enable low pressure interrupt
 * @param high Enable high pressure interrupt
 * @param latching Enable latching mode for interrupts
 * @return true If pressure interrupt configuration was successful
 * @return false If configuration failed
 */
bool Adafruit_LPS28::setPressureInterrupt(bool low, bool high, bool latching) {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits phe_bit(&interrupt_cfg, 1, 1);
  Adafruit_BusIO_RegisterBits ple_bit(&interrupt_cfg, 1, 2);
  Adafruit_BusIO_RegisterBits lir_bit(&interrupt_cfg, 1, 3);
  return phe_bit.write(high) && ple_bit.write(low) && lir_bit.write(latching);
}

/**
 * @brief Configures which conditions trigger the interrupt pin
 * 
 * @param drdy Enable data ready interrupt
 * @param drdy_pulse Enable pulsed mode for data ready interrupt
 * @param int_enable Enable pressure threshold interrupt
 * @param fifo_full Enable FIFO full interrupt
 * @param fifo_watermark Enable FIFO watermark interrupt
 * @param fifo_overrun Enable FIFO overrun interrupt
 * @return true If interrupt pin configuration was successful
 * @return false If configuration failed
 */
bool Adafruit_LPS28::setIntPinOutput(bool drdy, bool drdy_pulse,
                                     bool int_enable, bool fifo_full,
                                     bool fifo_watermark, bool fifo_overrun) {
  Adafruit_BusIO_Register ctrl_reg4(i2c_dev, LPS28_CTRL_REG4, 1);

  Adafruit_BusIO_RegisterBits drdy_pulse_bit(&ctrl_reg4, 1,
                                             6);          // Bit 6: DRDY_PLS
  Adafruit_BusIO_RegisterBits drdy_bit(&ctrl_reg4, 1, 5); // Bit 5: DRDY
  Adafruit_BusIO_RegisterBits int_enable_bit(&ctrl_reg4, 1, 4); // Bit 4: INT_EN
  Adafruit_BusIO_RegisterBits fifo_full_bit(&ctrl_reg4, 1,
                                            2); // Bit 2: INT_F_FULL
  Adafruit_BusIO_RegisterBits fifo_watermark_bit(&ctrl_reg4, 1,
                                                 1); // Bit 1: INT_F_WTM
  Adafruit_BusIO_RegisterBits fifo_overrun_bit(&ctrl_reg4, 1,
                                               0); // Bit 0: INT_F_OVR

  bool drdy_pulse_ok = drdy_pulse_bit.write(drdy_pulse);
  bool drdy_ok = drdy_bit.write(drdy);
  bool int_enable_ok = int_enable_bit.write(int_enable);
  bool fifo_full_ok = fifo_full_bit.write(fifo_full);
  bool fifo_watermark_ok = fifo_watermark_bit.write(fifo_watermark);
  bool fifo_overrun_ok = fifo_overrun_bit.write(fifo_overrun);

  return drdy_pulse_ok && drdy_ok && int_enable_ok && fifo_full_ok &&
         fifo_watermark_ok && fifo_overrun_ok;
}


/**
 * @brief Configures the FIFO operation mode
 * 
 * @param stop_on_watermark Stop collecting data when watermark is reached
 * @param mode FIFO operation mode from lps28_fifo_mode_t enum
 * @return true If FIFO configuration was successful
 * @return false If configuration failed
 */
bool Adafruit_LPS28::setFIFOmode(bool stop_on_watermark,
                                 lps28_fifo_mode_t mode) {
  Adafruit_BusIO_Register fifo_ctrl(i2c_dev, LPS28_FIFO_CTRL, 1);

  Adafruit_BusIO_RegisterBits stop_on_wtm_bit(&fifo_ctrl, 1,
                                              3); // Bit 3: STOP_ON_WTM
  Adafruit_BusIO_RegisterBits f_mode_bits(&fifo_ctrl, 3,
                                          0); // Bits 2:0 (F_MODE[2:0])

  bool stop_ok = stop_on_wtm_bit.write(stop_on_watermark);
  bool mode_ok = f_mode_bits.write(mode);

  return stop_ok && mode_ok;
}

/**
 * @brief Sets the FIFO watermark level
 * 
 * @param wtm Watermark level (0-127)
 * @return true If watermark was set successfully
 * @return false If setting failed
 */
bool Adafruit_LPS28::setFIFOWatermark(uint8_t wtm) {
  Adafruit_BusIO_Register fifo_wtm(i2c_dev, LPS28_FIFO_WTM, 1);
  return fifo_wtm.write(wtm);
}

/**
 * @brief Gets the current FIFO watermark level
 * 
 * @return uint8_t Current watermark level (0-127)
 */
uint8_t Adafruit_LPS28::getFIFOWatermark() {
  Adafruit_BusIO_Register fifo_wtm(i2c_dev, LPS28_FIFO_WTM, 1);
  return fifo_wtm.read();
}

/**
 * @brief Gets the current reference pressure value
 * 
 * @return int16_t Current reference pressure value
 */
int16_t Adafruit_LPS28::getReferencePressure() {
  Adafruit_BusIO_Register ref_p(i2c_dev, LPS28_REF_P, 2, LSBFIRST);
  return ref_p.read();
}

/**
 * @brief Sets the pressure offset value
 * 
 * @param offset Pressure offset value to set
 * @return true If offset was set successfully
 * @return false If setting failed
 */
bool Adafruit_LPS28::setPressureOffset(int16_t offset) {
  Adafruit_BusIO_Register rpds(i2c_dev, LPS28_RPDS, 2, LSBFIRST);
  return rpds.write(offset);
}

/**
 * @brief Gets the current pressure offset value
 * 
 * @return int16_t Current pressure offset value
 */
int16_t Adafruit_LPS28::getPressureOffset() {
  Adafruit_BusIO_Register rpds(i2c_dev, LPS28_RPDS, 2, LSBFIRST);
  return rpds.read();
}

/**
 * @brief Gets the source of the last interrupt
 * 
 * @return uint8_t Interrupt source register value
 *         Bit 7: AUTOZERO
 *         Bit 6: RESET_ARP
 *         Bit 5: AUTOREFP
 *         Bit 4: RESET_AZ
 *         Bit 3: IA
 *         Bit 2: PL
 *         Bit 1: PH
 *         Bit 0: BOOT
 */
uint8_t Adafruit_LPS28::getIntSource() {
  Adafruit_BusIO_Register int_source(i2c_dev, LPS28_INT_SOURCE, 1);
  return int_source.read();
}

/**
 * @brief Gets the number of unread samples in the FIFO
 * 
 * @return uint8_t Number of unread FIFO samples
 */
uint8_t Adafruit_LPS28::getFIFOunreadSamples() {
  Adafruit_BusIO_Register fifo_status1(i2c_dev, LPS28_FIFO_STATUS1, 1);
  return fifo_status1.read();
}

/**
 * @brief Gets the current sensor status
 * 
 * @return uint8_t Status register value
 *         Bit 3: P_OR (Pressure data overrun)
 *         Bit 2: T_OR (Temperature data overrun)
 *         Bit 1: P_DA (Pressure data available)
 *         Bit 0: T_DA (Temperature data available)
 */
uint8_t Adafruit_LPS28::getStatus() {
  Adafruit_BusIO_Register status_reg(i2c_dev, LPS28_STATUS, 1);
  return status_reg.read();
}

/**
 * @brief Gets the current pressure reading
 * 
 * @return float Pressure reading in hPa (hectopascals)
 * @note Resolution depends on FS_MODE:
 *       - FS_MODE = 0: 1 LSB = 1/4096 hPa
 *       - FS_MODE = 1: 1 LSB = 1/2048 hPa
 */
float Adafruit_LPS28::getPressure() {
  // Read the 3-byte PRESS_OUT register
  Adafruit_BusIO_Register pressure_out(i2c_dev, LPS28_PRESS_OUT, 3, LSBFIRST);
  int32_t raw_pressure = pressure_out.read();

  // Get full-scale mode using the getter
  bool fs_mode = getFullScaleMode();

  // Convert raw pressure value to hPa based on FS_MODE
  float pressure_hpa;
  if (fs_mode) {
    pressure_hpa = raw_pressure / 2048.0; // FS_MODE = 1, 1 LSB = 1/2048 hPa
  } else {
    pressure_hpa = raw_pressure / 4096.0; // FS_MODE = 0, 1 LSB = 1/4096 hPa
  }

  return pressure_hpa;
}

/**
 * @brief Gets the current temperature reading
 * 
 * @return float Temperature in degrees Celsius
 * @note Resolution is 0.01°C per LSB
 */
float Adafruit_LPS28::getTemperature() {
  // Read the 2-byte TEMP_OUT register (signed 16-bit, 2's complement)
  Adafruit_BusIO_Register temp_out(i2c_dev, LPS28_TEMP_OUT, 2, LSBFIRST);
  int16_t raw_temperature = (int16_t)temp_out.read();

  // Convert raw temperature to degrees Celsius
  float temperature_celsius = raw_temperature * 0.01; // 1 LSB = 0.01 deg C

  return temperature_celsius;
}

/**
 * @brief Gets the current FIFO status
 * 
 * @return uint8_t FIFO status register value
 *         Bit 7-6: FIFO_FULL_IA (FIFO full interrupt active)
 *         Bit 5: FIFO_OVR_IA (FIFO overrun interrupt active)
 *         Bit 4: FIFO_WTM_IA (FIFO watermark interrupt active)
 *         Bit 3-0: Reserved
 */
uint8_t Adafruit_LPS28::getFIFOstatus() {
  Adafruit_BusIO_Register fifo_status2(i2c_dev, LPS28_FIFO_STATUS2, 1);
  return fifo_status2.read();
}

/**
 * @brief Reads the next pressure value from the FIFO buffer
 * 
 * @return float Pressure reading from FIFO in hPa (hectopascals)
 * @note Resolution depends on FS_MODE:
 *       - FS_MODE = 0: 1 LSB = 1/4096 hPa
 *       - FS_MODE = 1: 1 LSB = 1/2048 hPa
 */
float Adafruit_LPS28::getFIFOpressure() {
  // Read the 3-byte FIFO_DATA_OUT_PRESS_XL register
  Adafruit_BusIO_Register fifo_pressure_out(
      i2c_dev, LPS28_FIFO_DATA_OUT_PRESS_XL, 3, LSBFIRST);
  int32_t raw_pressure = fifo_pressure_out.read();

  // Get full-scale mode using the getter
  bool fs_mode = getFullScaleMode();

  // Convert raw pressure value to hPa based on FS_MODE
  float pressure_hpa;
  if (fs_mode) {
    pressure_hpa = raw_pressure / 2048.0; // FS_MODE = 1, 1 LSB = 1/2048 hPa
  } else {
    pressure_hpa = raw_pressure / 4096.0; // FS_MODE = 0, 1 LSB = 1/4096 hPa
  }

  return pressure_hpa;
}
