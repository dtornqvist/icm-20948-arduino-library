#include "Arduino.h"
#include "ICM20948.h"

/* ICM20948 object, input the I2C bus and address */
ICM20948::ICM20948(TwoWire &bus,uint8_t address) {
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
  if(writeRegister(UB0_PWR_MGMNT_1, UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO) < 0){
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
  if(writeRegister(UB0_PWR_MGMNT_2, UB0_PWR_MGMNT_2_SEN_ENABLE) < 0){
    return -6;
  }
  // setting accel range to 16G as default
  /*if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
    return -7;
  }
  _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
  _accelRange = ACCEL_RANGE_16G;
  // setting the gyro range to 2000DPS as default
  if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
    return -8;
  }
  _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
  _gyroRange = GYRO_RANGE_2000DPS;
  // setting bandwidth to 184Hz as default
  if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ 
    return -9;
  } 
  if(writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
    return -10;
  }
  _bandwidth = DLPF_BANDWIDTH_184HZ;
  // setting the sample rate divider to 0 as default
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

/* gets the WHO_AM_I register value, expected to be 0xEA */
int ICM20948::whoAmI(){
  // read the WHO AM I register
  if (readRegisters(UB0_WHO_AM_I, 1, _buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
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