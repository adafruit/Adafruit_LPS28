#include "Adafruit_LPS28.h"

Adafruit_LPS28::Adafruit_LPS28() {}

bool Adafruit_LPS28::begin(TwoWire *theWire, uint8_t i2c_addr) {
  i2c_dev = new Adafruit_I2CDevice(i2c_addr, theWire);
  if (!i2c_dev->begin()) {
    return false;
  }

  Adafruit_BusIO_Register whoami_reg(i2c_dev, LPS28_WHOAMI_REG, 1);
  return (whoami_reg.read() == LPS28_WHOAMI_EXPECTED);
}

bool Adafruit_LPS28::setThresholdPressure(uint16_t threshold) {
  Adafruit_BusIO_Register ths_p(i2c_dev, LPS28_THS_P, 2, LSBFIRST);
  return ths_p.write(threshold);
}

uint16_t Adafruit_LPS28::getThresholdPressure() {
  Adafruit_BusIO_Register ths_p(i2c_dev, LPS28_THS_P, 2, LSBFIRST);
  return ths_p.read();
}

bool Adafruit_LPS28::setDataRate(lps28_odr_t odr) {
  Adafruit_BusIO_Register ctrl_reg1(i2c_dev, LPS28_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits odr_bits(&ctrl_reg1, 4, 3);
  return odr_bits.write(odr);
}

lps28_odr_t Adafruit_LPS28::getDataRate() {
  Adafruit_BusIO_Register ctrl_reg1(i2c_dev, LPS28_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits odr_bits(&ctrl_reg1, 4, 3);
  return (lps28_odr_t)odr_bits.read();
}

bool Adafruit_LPS28::setAveraging(lps28_avg_t avg) {
  Adafruit_BusIO_Register ctrl_reg1(i2c_dev, LPS28_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits avg_bits(&ctrl_reg1, 3, 0);
  return avg_bits.write(avg);
}

lps28_avg_t Adafruit_LPS28::getAveraging() {
  Adafruit_BusIO_Register ctrl_reg1(i2c_dev, LPS28_CTRL_REG1, 1);
  Adafruit_BusIO_RegisterBits avg_bits(&ctrl_reg1, 3, 0);
  return (lps28_avg_t)avg_bits.read();
}

bool Adafruit_LPS28::rebootMemory() {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits boot_bit(&ctrl_reg2, 1, 7);
  return boot_bit.write(true);
}

bool Adafruit_LPS28::setFullScaleMode(bool mode) {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits fs_mode_bit(&ctrl_reg2, 1, 6);
  return fs_mode_bit.write(mode);
}

bool Adafruit_LPS28::getFullScaleMode() {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits fs_mode_bit(&ctrl_reg2, 1,
                                          6); // FS_MODE bit is bit 6
  return fs_mode_bit.read();
}

bool Adafruit_LPS28::setLowPassODR9(bool enable) {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits lpfp_cfg_bit(&ctrl_reg2, 1, 5);
  return lpfp_cfg_bit.write(enable);
}

bool Adafruit_LPS28::reset() {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits swreset_bit(&ctrl_reg2, 1, 2);
  return swreset_bit.write(true);
}

bool Adafruit_LPS28::triggerOneShot() {
  Adafruit_BusIO_Register ctrl_reg2(i2c_dev, LPS28_CTRL_REG2, 1);
  Adafruit_BusIO_RegisterBits oneshot_bit(&ctrl_reg2, 1, 0);
  return oneshot_bit.write(true);
}

bool Adafruit_LPS28::setInterruptPin(bool polarity, bool openDrain) {
  Adafruit_BusIO_Register ctrl_reg3(i2c_dev, LPS28_CTRL_REG3, 1);
  Adafruit_BusIO_RegisterBits int_h_l_bit(&ctrl_reg3, 1, 3);
  Adafruit_BusIO_RegisterBits pp_od_bit(&ctrl_reg3, 1, 1);
  return int_h_l_bit.write(polarity) && pp_od_bit.write(openDrain);
}

bool Adafruit_LPS28::setSDAPullup(bool enable) {
  Adafruit_BusIO_Register if_ctrl(i2c_dev, LPS28_IF_CTRL, 1);
  Adafruit_BusIO_RegisterBits sda_pu_bit(&if_ctrl, 1, 4);
  return sda_pu_bit.write(enable);
}

bool Adafruit_LPS28::setINTPulldown(bool enable) {
  Adafruit_BusIO_Register if_ctrl(i2c_dev, LPS28_IF_CTRL, 1);
  Adafruit_BusIO_RegisterBits int_pd_dis_bit(&if_ctrl, 1, 2);
  return int_pd_dis_bit.write(!enable);
}

bool Adafruit_LPS28::setAutoReferencePressure(bool enable) {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits autozero_bit(&interrupt_cfg, 1, 7);
  return autozero_bit.write(enable);
}

bool Adafruit_LPS28::getAutoReferencePressure() {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits autozero_bit(&interrupt_cfg, 1, 7);
  return autozero_bit.read();
}

bool Adafruit_LPS28::resetAutoReferencePressure() {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits reset_arp_bit(&interrupt_cfg, 1, 6);
  return reset_arp_bit.write(true);
}

bool Adafruit_LPS28::setAutoZero(bool enable) {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits autozero_bit(&interrupt_cfg, 1, 5);
  return autozero_bit.write(enable);
}

bool Adafruit_LPS28::getAutoZero() {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits autozero_bit(&interrupt_cfg, 1, 5);
  return autozero_bit.read();
}

bool Adafruit_LPS28::resetAutoZero() {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits reset_az_bit(&interrupt_cfg, 1, 4);
  return reset_az_bit.write(true);
}

bool Adafruit_LPS28::setPressureInterrupt(bool low, bool high, bool latching) {
  Adafruit_BusIO_Register interrupt_cfg(i2c_dev, LPS28_INTERRUPT_CFG, 1);
  Adafruit_BusIO_RegisterBits phe_bit(&interrupt_cfg, 1, 1);
  Adafruit_BusIO_RegisterBits ple_bit(&interrupt_cfg, 1, 2);
  Adafruit_BusIO_RegisterBits lir_bit(&interrupt_cfg, 1, 3);
  return phe_bit.write(high) && ple_bit.write(low) && lir_bit.write(latching);
}

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

bool Adafruit_LPS28::setFIFOWatermark(uint8_t wtm) {
  Adafruit_BusIO_Register fifo_wtm(i2c_dev, LPS28_FIFO_WTM, 1);
  return fifo_wtm.write(wtm);
}

uint8_t Adafruit_LPS28::getFIFOWatermark() {
  Adafruit_BusIO_Register fifo_wtm(i2c_dev, LPS28_FIFO_WTM, 1);
  return fifo_wtm.read();
}

int16_t Adafruit_LPS28::getReferencePressure() {
  Adafruit_BusIO_Register ref_p(i2c_dev, LPS28_REF_P, 2, LSBFIRST);
  return ref_p.read();
}

bool Adafruit_LPS28::setPressureOffset(int16_t offset) {
  Adafruit_BusIO_Register rpds(i2c_dev, LPS28_RPDS, 2, LSBFIRST);
  return rpds.write(offset);
}

int16_t Adafruit_LPS28::getPressureOffset() {
  Adafruit_BusIO_Register rpds(i2c_dev, LPS28_RPDS, 2, LSBFIRST);
  return rpds.read();
}

uint8_t Adafruit_LPS28::getIntSource() {
  Adafruit_BusIO_Register int_source(i2c_dev, LPS28_INT_SOURCE, 1);
  return int_source.read();
}

uint8_t Adafruit_LPS28::getFIFOunreadSamples() {
  Adafruit_BusIO_Register fifo_status1(i2c_dev, LPS28_FIFO_STATUS1, 1);
  return fifo_status1.read();
}

uint8_t Adafruit_LPS28::getStatus() {
  Adafruit_BusIO_Register status_reg(i2c_dev, LPS28_STATUS, 1);
  return status_reg.read();
}

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

float Adafruit_LPS28::getTemperature() {
  // Read the 2-byte TEMP_OUT register (signed 16-bit, 2's complement)
  Adafruit_BusIO_Register temp_out(i2c_dev, LPS28_TEMP_OUT, 2, LSBFIRST);
  int16_t raw_temperature = (int16_t)temp_out.read();

  // Convert raw temperature to degrees Celsius
  float temperature_celsius = raw_temperature * 0.01; // 1 LSB = 0.01 °C

  return temperature_celsius;
}

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
