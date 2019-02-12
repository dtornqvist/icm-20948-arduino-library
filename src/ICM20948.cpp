#include "Arduino.h"
#include "ICM20948.h"

/* ICM20948 object, input the I2C bus and address */
ICM20948::ICM20948(TwoWire &bus, uint8_t address) {
  _i2c = &bus; // I2C bus
  _address = address; // I2C address
}

/* starts communication with the MPU-9250 */
int ICM20948::begin() {
  // starting the I2C bus
  _i2c->begin();
  // setting the I2C clock
  _i2c->setClock(_i2cRate);

  // select clock source to auto
  if (writeRegister(UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO) < 0) {
    return -1;
  }
  // enable I2C master mode
  /*if(writeRegister(USER_CTRL, USER_CTRL_I2C_MST_EN) < 0){
    return -2;
  }
  // set the I2C bus speed to 400 kHz
  if(writeRegister(I2C_MST_CTRL, I2C_MST_CLK) < 0){
    return -3;
  }*/
  // set AK8963 to Power Down
  //writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the ICM20948
  //writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  //delay(1);
  // reset the AK8963
  //writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
  // select clock source to auto
  /*if(writeRegister(PWR_MGMNT_1, PWR_MGMNT_1_CLOCK_SEL_AUTO) < 0){
    return -4;
  }*/
  // check the WHO AM I byte, expected value is 0xEA
  if (whoAmI() != 0xEA) {
    return -5;
  }
  // enable accelerometer and gyro
  if (writeRegister(UB0_PWR_MGMNT_2, UB0_PWR_MGMNT_2_SEN_ENABLE) < 0) {
    return -6;
  }
  // setting accel range to 16G as default
  if (configAccel(ACCEL_RANGE_16G, ACCEL_DLPF_BANDWIDTH_246HZ) < 0) {
    return -7;
  }
  // setting the gyro range to 2000DPS as default
  if (configGyro(GYRO_RANGE_2000DPS, GYRO_DLPF_BANDWIDTH_197HZ) < 0) {
    return -8;
  }
  /*
  if(writeRegister(SMPDIV,0x00) < 0){ 
    return -11;
  } 
  _srd = 0;
  // enable I2C master mode
  /*if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
  	return -12;
  }
	// set the I2C bus speed to 400 kHz
	if( writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
		return -13;
	}
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 72 ){
    return -14;
	}
  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  /*if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -15;
  }
  delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if(writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
    return -16;
  }
  delay(100); // long wait between AK8963 mode changes
  // read the AK8963 ASA registers and compute magnetometer scale factors
  readAK8963Registers(AK8963_ASA,3,_buffer);
  _magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla 
  // set AK8963 to Power Down
  if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -17;
  }
  delay(100); // long wait between AK8963 mode changes  
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
    return -18;
  }
  delay(100); // long wait between AK8963 mode changes
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -19;
  }       
  // instruct the ICM20948 to get 7 bytes of data from the AK8963 at the sample rate
  readAK8963Registers(AK8963_HXL,7,_buffer);
  // estimate gyro bias
  if (calibrateGyro() < 0) {
    return -20;
  }*/
  // successful init, return 1
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

int ICM20948::changeUserBank(UserBank userBank) {
	if (userBank == _currentUserBank) {
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