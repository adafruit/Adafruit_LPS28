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
