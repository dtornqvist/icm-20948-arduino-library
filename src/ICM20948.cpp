/*
ICM20948.cpp

Copyright (c) 2019 David Törnqvist

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "Arduino.h"
#include "ICM20948.h"

/* ICM20948 object, input the I2C bus and address */
ICM20948::ICM20948(TwoWire &bus, uint8_t address) {
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
}

/* starts communication with the ICM-20948 */
int ICM20948::begin() {
  _i2c->begin(); // starting the I2C bus
  _i2c->setClock(_i2cRate); // setting the I2C clock

  if (changeUserBank(USER_BANK_0, true) < 0) { // Make sure that the user bank selection is in sync
  	return -1;
  }
  if (selectAutoClockSource() < 0) { // TODO: Why set clock source here? It is resetted anyway...
    return -1;
  }
  // enable I2C master mode
  if(enableI2cMaster() < 0){
    return -2;
  }
  if (powerDownMag() < 0) {
  	return -3;
  }
  reset(); // reset the ICM20948. Don't check return value as a reset clears the register and can't be verified.
  delay(1); // wait for ICM-20948 to come back up
  resetMag(); // Don't check return value as a reset clears the register and can't be verified.
  if (selectAutoClockSource() < 0) {
    return -6;
  }
  if (whoAmI() != ICM20948_WHO_AM_I) {
    return -7;
  }
  if (enableAccelGyro() < 0) {
    return -8;
  }
  if (configAccel(ACCEL_RANGE_16G, ACCEL_DLPF_BANDWIDTH_246HZ) < 0) {
    return -9;
  }
  if (configGyro(GYRO_RANGE_2000DPS, GYRO_DLPF_BANDWIDTH_197HZ) < 0) {
    return -10;
  }
  if (setGyroSrd(0) < 0) { 
    return -11;
  } 
  if (setAccelSrd(0) < 0) { 
    return -12;
  }
  if(enableI2cMaster() < 0) {
    return -13;
  }
	if(whoAmIMag() != MAG_AK09916_WHO_AM_I ) {
    return -14;
	}
  if(configMag() < 0){
    return -18;
  }
  if(selectAutoClockSource() < 0) { // TODO: Why do this again here?
    return -19;
  }       
  readMagRegisters(MAG_HXL, MAG_DATA_LENGTH, _buffer); // instruct the ICM20948 to get data from the magnetometer at the sample rate
  return 1;
}

int ICM20948::enableI2cMaster() {
	if (changeUserBank(USER_BANK_0) < 0) {
    return -1;
  }
  if (writeRegister(UB0_USER_CTRL, UB0_USER_CTRL_I2C_MST_EN) < 0) {
    return -2;
  }
  if (changeUserBank(USER_BANK_3) < 0) {
    return -3;
  }
  if(writeRegister(UB3_I2C_MST_CTRL, UB3_I2C_MST_CTRL_CLK_400KHZ) < 0){
    return -4;
  }
  return 1;
}

/* enables the data ready interrupt */
int ICM20948::enableDataReadyInterrupt() {
	if (changeUserBank(USER_BANK_0) < 0) {
    return -1;
  }
  if (writeRegister(UB0_INT_PIN_CFG, UB0_INT_PIN_CFG_HIGH_50US) < 0) { // setup interrupt, 50 us pulse
    return -2;
  }  
  if (writeRegister(UB0_INT_ENABLE_1, UB0_INT_ENABLE_1_RAW_RDY_EN) < 0) { // set to data ready
    return -3;
  }
  return 1;
}

/* disables the data ready interrupt */
int ICM20948::disableDataReadyInterrupt() {
  if (changeUserBank(USER_BANK_0) < 0) {
    return -1;
  }
  if (writeRegister(UB0_INT_ENABLE_1, UB0_INT_ENABLE_1_DIS) < 0) { // disable interrupt
    return -1;
  }  
  return 1;
}

int ICM20948::reset() {
	if (changeUserBank(USER_BANK_0) < 0) {
    return -1;
  }
  if (writeRegister(UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_DEV_RESET) < 0) {
  	return -2;
  }
  return 1;
}

int ICM20948::selectAutoClockSource() {
	if (changeUserBank(USER_BANK_0) < 0 || writeRegister(UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO) < 0) {
    return -1;
  }
  return 1;
}

int ICM20948::enableAccelGyro() {
	if (changeUserBank(USER_BANK_0) < 0 || writeRegister(UB0_PWR_MGMNT_2, UB0_PWR_MGMNT_2_SEN_ENABLE) < 0) {
    return -1;
  }
  return 1;
}

int ICM20948::configAccel(AccelRange range, AccelDlpfBandwidth bandwidth) {
	if (changeUserBank(USER_BANK_2) < 0) {
  	return -1;
  }
  uint8_t accelRangeRegValue = 0x00;
  float accelScale = 0.0f;
  switch(range) {
    case ACCEL_RANGE_2G: {
      accelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_2G;
      accelScale = G * 2.0f/accRawScaling; // setting the accel scale to 2G
      break; 
    }
    case ACCEL_RANGE_4G: {
      accelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_4G;
      accelScale = G * 4.0f/accRawScaling; // setting the accel scale to 4G
      break;
    }
    case ACCEL_RANGE_8G: {
      accelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_8G;
      accelScale = G * 8.0f/accRawScaling; // setting the accel scale to 8G
      break;
    }
    case ACCEL_RANGE_16G: {
      accelRangeRegValue = UB2_ACCEL_CONFIG_FS_SEL_16G;
      accelScale = G * 16.0f/accRawScaling; // setting the accel scale to 16G
      break;
    }
  }
  uint8_t dlpfRegValue = 0x00;
  switch(bandwidth) {
  	case ACCEL_DLPF_BANDWIDTH_1209HZ: dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_1209HZ; break;
  	case ACCEL_DLPF_BANDWIDTH_246HZ: dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_246HZ; break;
  	case ACCEL_DLPF_BANDWIDTH_111HZ: dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_111HZ; break;
  	case ACCEL_DLPF_BANDWIDTH_50HZ: dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_50HZ; break;
  	case ACCEL_DLPF_BANDWIDTH_24HZ: dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_24HZ; break;
  	case ACCEL_DLPF_BANDWIDTH_12HZ: dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_12HZ; break;
  	case ACCEL_DLPF_BANDWIDTH_6HZ: dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_6HZ; break;
  	case ACCEL_DLPF_BANDWIDTH_473HZ: dlpfRegValue = UB2_ACCEL_CONFIG_DLPFCFG_473HZ; break;
  }
  if (writeRegister(UB2_ACCEL_CONFIG, accelRangeRegValue | dlpfRegValue) < 0) {
		return -1;
  }
  _accelScale = accelScale;
  _accelRange = range;
  _accelBandwidth = bandwidth;
  return 1;
}

/* sets the gyro full scale range to values other than default */
int ICM20948::configGyro(GyroRange range, GyroDlpfBandwidth bandwidth) {
  if (changeUserBank(USER_BANK_2) < 0) {
  	return -1;
  }
  uint8_t gyroConfigRegValue = 0x00;
  float gyroScale = 0x00;
  switch(range) {
    case GYRO_RANGE_250DPS: {
    	gyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_250DPS;
      gyroScale = 250.0f/gyroRawScaling * _d2r; // setting the gyro scale to 250DPS
      break;
    }
    case GYRO_RANGE_500DPS: {
      gyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_500DPS;
      gyroScale = 500.0f/gyroRawScaling * _d2r; // setting the gyro scale to 500DPS
      break;  
    }
    case GYRO_RANGE_1000DPS: {
      gyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_1000DPS;
      gyroScale = 1000.0f/gyroRawScaling * _d2r; // setting the gyro scale to 1000DPS
      break;
    }
    case GYRO_RANGE_2000DPS: {
      gyroConfigRegValue = UB2_GYRO_CONFIG_1_FS_SEL_2000DPS;
      gyroScale = 2000.0f/gyroRawScaling * _d2r; // setting the gyro scale to 2000DPS
      break;
    }
  }
  uint8_t dlpfRegValue = 0x00;
  switch(bandwidth) {
  	case GYRO_DLPF_BANDWIDTH_12106HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_12106HZ; break;
  	case GYRO_DLPF_BANDWIDTH_197HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_197HZ; break;
  	case GYRO_DLPF_BANDWIDTH_152HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_152HZ; break;
  	case GYRO_DLPF_BANDWIDTH_120HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_120HZ; break;
  	case GYRO_DLPF_BANDWIDTH_51HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_51HZ; break;
  	case GYRO_DLPF_BANDWIDTH_24HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_24HZ; break;
  	case GYRO_DLPF_BANDWIDTH_12HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_12HZ; break;
  	case GYRO_DLPF_BANDWIDTH_6HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_6HZ; break;
  	case GYRO_DLPF_BANDWIDTH_361HZ: dlpfRegValue = UB2_GYRO_CONFIG_1_DLPFCFG_361HZ; break;
  }
  if (writeRegister(UB2_GYRO_CONFIG_1, gyroConfigRegValue | dlpfRegValue) < 0) {
  	return -1;
	}
	_gyroScale = gyroScale;
  _gyroRange = range;
  _gyroBandwidth = bandwidth;
  return 1;
}

int ICM20948::configMag() { // TODO: Add possibility to use other modes
	if (writeMagRegister(MAG_CNTL2, MAG_CNTL2_MODE_100HZ) < 0) {
		return -1;
	}
	return 1;
}

int ICM20948::setGyroSrd(uint8_t srd) {
	if (changeUserBank(USER_BANK_2) < 0 || writeRegister(UB2_GYRO_SMPLRT_DIV, srd) < 0) {
  	return -1;
	}
	_gyroSrd = srd;
	return 1;
}

int ICM20948::setAccelSrd(uint16_t srd) {
	if (changeUserBank(USER_BANK_2) < 0) {
  	return -1;
  }
  uint8_t srdHigh = srd >> 8 & 0x0F; // Only last 4 bits can be set
  if (writeRegister(UB2_ACCEL_SMPLRT_DIV_1, srdHigh) < 0) {
  	return -1;
	}
	uint8_t srdLow = srd & 0x0F; // Only last 4 bits can be set
  if (writeRegister(UB2_ACCEL_SMPLRT_DIV_2, srdLow) < 0) {
  	return -1;
	}
	_accelSrd = srd;
	return 1;
}

/* reads the most current data from MPU9250 and stores in buffer */
int ICM20948::readSensor() {
	if (changeUserBank(USER_BANK_0) < 0) {
  	return -1;
  }
 	if (readRegisters(UB0_ACCEL_XOUT_H, 20, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];  
  _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _gxcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
  _gycounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gzcounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _tcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];
  _hxcounts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
  _hycounts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
  _hzcounts = (((int16_t)_buffer[19]) << 8) | _buffer[18];
  // transform and convert to float values
  _ax = (((float)_axcounts * _accelScale) - _axb)*_axs;
  _ay = (((float)_aycounts * _accelScale) - _ayb)*_ays;
  _az = (((float)_azcounts * _accelScale) - _azb)*_azs;
  _gx = ((float)_gxcounts * _gyroScale) - _gxb;
  _gy = ((float)_gycounts * _gyroScale) - _gyb;
  _gz = ((float)_gzcounts * _gyroScale) - _gzb;
  _t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
  _hx = (((float)(tX[0]*_hxcounts + tX[1]*_hycounts + tX[2]*_hzcounts) * _magScale) - _hxb)*_hxs;
  _hy = (((float)(tY[0]*_hxcounts + tY[1]*_hycounts + tY[2]*_hzcounts) * _magScale) - _hyb)*_hys;
  _hz = (((float)(tZ[0]*_hxcounts + tZ[1]*_hycounts + tZ[2]*_hzcounts) * _magScale) - _hzb)*_hzs;
  return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
float ICM20948::getAccelX_mss() {
  return _ax;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
float ICM20948::getAccelY_mss() {
  return _ay;
}

/* returns the accelerometer measurement in the z direction, m/s/s */
float ICM20948::getAccelZ_mss() {
  return _az;
}

/* returns the gyroscope measurement in the x direction, rad/s */
float ICM20948::getGyroX_rads() {
  return _gx;
}

/* returns the gyroscope measurement in the y direction, rad/s */
float ICM20948::getGyroY_rads() {
  return _gy;
}

/* returns the gyroscope measurement in the z direction, rad/s */
float ICM20948::getGyroZ_rads() {
  return _gz;
}

/* returns the magnetometer measurement in the x direction, uT */
float ICM20948::getMagX_uT() {
  return _hx;
}

/* returns the magnetometer measurement in the y direction, uT */
float ICM20948::getMagY_uT() {
  return _hy;
}

/* returns the magnetometer measurement in the z direction, uT */
float ICM20948::getMagZ_uT() {
  return _hz;
}

/* returns the die temperature, C */
float ICM20948::getTemperature_C() {
  return _t;
}

/* gets the WHO_AM_I register value, expected to be 0xEA */
int ICM20948::whoAmI() {
	if (changeUserBank(USER_BANK_0) < 0) {
  	return -1;
  }
  // read the WHO AM I register
  if (readRegisters(UB0_WHO_AM_I, 1, _buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}

int ICM20948::whoAmIMag() {
	if (readMagRegisters(MAG_WHO_AM_I, 2, _buffer) < 0) {
    return -1;
  }
  return (_buffer[0] << 8) + _buffer[1];
}

int ICM20948::powerDownMag() {
	if (writeMagRegister(MAG_CNTL2, MAG_CNTL2_POWER_DOWN) < 0) {
		return -1;
	}
	return 1;
}

int ICM20948::resetMag() {
	if (writeMagRegister(MAG_CNTL3, MAG_CNTL3_RESET) < 0) {
		return -1;
	}
	return 1;
}

int ICM20948::changeUserBank(UserBank userBank) {
	return changeUserBank(userBank, false);
}

int ICM20948::changeUserBank(UserBank userBank, bool force) {
	if (!force && userBank == _currentUserBank) {
		return 2; // No need to change
	}
	uint8_t userBankRegValue = 0x00;
	switch(userBank) {
    case USER_BANK_0: {
    	userBankRegValue = REG_BANK_SEL_USER_BANK_0;
  		break;
    }
    case USER_BANK_1: {
    	userBankRegValue = REG_BANK_SEL_USER_BANK_1;
  		break;
    }
    case USER_BANK_2: {
    	userBankRegValue = REG_BANK_SEL_USER_BANK_2;
  		break;
    }
    case USER_BANK_3: {
    	userBankRegValue = REG_BANK_SEL_USER_BANK_3;
  		break;
    }
  }
  if (writeRegister(REG_BANK_SEL, userBankRegValue) < 0) {
  	return -1;
  }
  _currentUserBank = userBank;
  return 1;
}

int ICM20948::writeMagRegister(uint8_t subAddress, uint8_t data) {
	if (changeUserBank(USER_BANK_3) < 0) {
  	return -1;
  }
	if (writeRegister(UB3_I2C_SLV0_ADDR, MAG_AK09916_I2C_ADDR) < 0) {
    return -2;
  }
  // set the register to the desired magnetometer sub address 
	if (writeRegister(UB3_I2C_SLV0_REG, subAddress) < 0) {
    return -3;
  }
  // store the data for write
	if (writeRegister(UB3_I2C_SLV0_DO, data) < 0) {
    return -4;
  }
  // enable I2C and send 1 byte
	if (writeRegister(UB3_I2C_SLV0_CTRL, UB3_I2C_SLV0_CTRL_EN | (uint8_t)1) < 0) {
    return -5;
  }
	// read the register and confirm
	if (readMagRegisters(subAddress, 1, _buffer) < 0) {
    return -6;
  }
	if(_buffer[0] != data) {
  	return -7;
  }
  return 1;
}

int ICM20948::readMagRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
	if (changeUserBank(USER_BANK_3) < 0) {
  	return -1;
  }
	if (writeRegister(UB3_I2C_SLV0_ADDR, MAG_AK09916_I2C_ADDR | UB3_I2C_SLV0_ADDR_READ_FLAG) < 0) {
    return -2;
  }
  // set the register to the desired magnetometer sub address
	if (writeRegister(UB3_I2C_SLV0_REG, subAddress) < 0) {
    return -3;
  }
  // enable I2C and request the bytes
	if (writeRegister(UB3_I2C_SLV0_CTRL, UB3_I2C_SLV0_CTRL_EN | count) < 0) {
    return -4;
  }
	delay(1); // takes some time for these registers to fill
  // read the bytes off the ICM-20948 EXT_SLV_SENS_DATA registers
  if (changeUserBank(USER_BANK_0) < 0) {
  	return -5;
  }
	_status = readRegisters(UB0_EXT_SLV_SENS_DATA_00, count, dest); 
  return _status;
}

/* writes a byte to ICM20948 register given a register address and data */
int ICM20948::writeRegister(uint8_t subAddress, uint8_t data) {
  /* write data to device */
  _i2c->beginTransmission(_address); // open the device
  _i2c->write(subAddress); // write the register address
	_i2c->write(data); // write the data
	_i2c->endTransmission();

  delay(10);
  
  /* read back the register */
  readRegisters(subAddress, 1, _buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}

/* reads registers from ICM20948 given a starting register address, number of bytes, and a pointer to store data */
int ICM20948::readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest) {
  _i2c->beginTransmission(_address); // open the device
  _i2c->write(subAddress); // specify the starting register address
  _i2c->endTransmission(false);
  _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
  if (_numBytes == count) {
    for(uint8_t i = 0; i < count; i++){ 
      dest[i] = _i2c->read();
    }
    return 1;
  } else {
    return -1;
  }
}