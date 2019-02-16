/*
ICM20948.h

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

#ifndef ICM20948_h
#define ICM20948_h

#include "Arduino.h"
#include "Wire.h"    // I2C library

class ICM20948 {
  public:
  	enum GyroRange
    {
      GYRO_RANGE_250DPS,
      GYRO_RANGE_500DPS,
      GYRO_RANGE_1000DPS,
      GYRO_RANGE_2000DPS
    };
    enum AccelRange
    {
      ACCEL_RANGE_2G,
      ACCEL_RANGE_4G,
      ACCEL_RANGE_8G,
      ACCEL_RANGE_16G    
    };
    enum AccelDlpfBandwidth
    {
    	ACCEL_DLPF_BANDWIDTH_1209HZ,
      ACCEL_DLPF_BANDWIDTH_246HZ,
      ACCEL_DLPF_BANDWIDTH_111HZ,
      ACCEL_DLPF_BANDWIDTH_50HZ,
      ACCEL_DLPF_BANDWIDTH_24HZ,
      ACCEL_DLPF_BANDWIDTH_12HZ,
      ACCEL_DLPF_BANDWIDTH_6HZ,
      ACCEL_DLPF_BANDWIDTH_473HZ
    };
    enum GyroDlpfBandwidth
    {
    	GYRO_DLPF_BANDWIDTH_12106HZ,
      GYRO_DLPF_BANDWIDTH_197HZ,
      GYRO_DLPF_BANDWIDTH_152HZ,
      GYRO_DLPF_BANDWIDTH_120HZ,
      GYRO_DLPF_BANDWIDTH_51HZ,
      GYRO_DLPF_BANDWIDTH_24HZ,
      GYRO_DLPF_BANDWIDTH_12HZ,
      GYRO_DLPF_BANDWIDTH_6HZ,
      GYRO_DLPF_BANDWIDTH_361HZ
    };
    enum LpAccelOdr
    {
      LP_ACCEL_ODR_0_24HZ = 0,
      LP_ACCEL_ODR_0_49HZ = 1,
      LP_ACCEL_ODR_0_98HZ = 2,
      LP_ACCEL_ODR_1_95HZ = 3,
      LP_ACCEL_ODR_3_91HZ = 4,
      LP_ACCEL_ODR_7_81HZ = 5,
      LP_ACCEL_ODR_15_63HZ = 6,
      LP_ACCEL_ODR_31_25HZ = 7,
      LP_ACCEL_ODR_62_50HZ = 8,
      LP_ACCEL_ODR_125HZ = 9,
      LP_ACCEL_ODR_250HZ = 10,
      LP_ACCEL_ODR_500HZ = 11
    };
    enum UserBank
    {
    	USER_BANK_0,
    	USER_BANK_1,
    	USER_BANK_2,
    	USER_BANK_3,
    };
  	ICM20948(TwoWire &bus, uint8_t address);
  	int begin();
    int configAccel(AccelRange range, AccelDlpfBandwidth bandwidth);
    int configGyro(GyroRange range, GyroDlpfBandwidth bandwidth);
    int configMag();
    int setGyroSrd(uint8_t srd);
    int setAccelSrd(uint16_t srd);
    int enableDataReadyInterrupt();
    int disableDataReadyInterrupt();
    int readSensor();
    float getAccelX_mss();
    float getAccelY_mss();
    float getAccelZ_mss();
    float getGyroX_rads();
    float getGyroY_rads();
    float getGyroZ_rads();
    float getMagX_uT();
    float getMagY_uT();
    float getMagZ_uT();
    float getTemperature_C();
  protected:
    // i2c
    uint8_t _address;
    TwoWire *_i2c;
    const uint32_t _i2cRate = 400000; // 400 kHz
    size_t _numBytes; // number of bytes received from I2C
    // track success of interacting with sensor
    int _status;
    // buffer for reading from sensor
    uint8_t _buffer[21];
    // data counts
    int16_t _axcounts,_aycounts,_azcounts;
    int16_t _gxcounts,_gycounts,_gzcounts;
    int16_t _hxcounts,_hycounts,_hzcounts;
    int16_t _tcounts;
    // data buffer
    float _ax, _ay, _az;
    float _gx, _gy, _gz;
    float _hx, _hy, _hz;
    float _t;
    // wake on motion
    uint8_t _womThreshold;
    // scale factors
    float _accelScale;
    float _gyroScale;
    const float _tempScale = 333.87f;
    const float _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    AccelDlpfBandwidth _accelBandwidth;
    GyroDlpfBandwidth _gyroBandwidth;
    UserBank _currentUserBank = USER_BANK_0;
    uint8_t _gyroSrd;
    uint16_t _accelSrd;
    // gyro bias estimation
    size_t _numSamples = 100;
    double _gxbD, _gybD, _gzbD;
    float _gxb, _gyb, _gzb;
    // accel bias and scale factor estimation
    double _axbD, _aybD, _azbD;
    float _axmax, _aymax, _azmax;
    float _axmin, _aymin, _azmin;
    float _axb, _ayb, _azb;
    float _axs = 1.0f;
    float _ays = 1.0f;
    float _azs = 1.0f;
    // magnetometer bias and scale factor estimation
    uint16_t _maxCounts = 1000;
    float _deltaThresh = 0.3f;
    uint8_t _coeff = 8;
    uint16_t _counter;
    float _framedelta, _delta;
    float _hxfilt, _hyfilt, _hzfilt;
    float _hxmax, _hymax, _hzmax;
    float _hxmin, _hymin, _hzmin;
    float _hxb, _hyb, _hzb;
    float _hxs = 1.0f;
    float _hys = 1.0f;
    float _hzs = 1.0f;
    float _avgs;

    // transformation matrix
    /* transform the magnetometer values to match the coordinate system of the IMU */
    const int16_t tX[3] = {1,  0,  0}; 
    const int16_t tY[3] = {0, -1,  0};
    const int16_t tZ[3] = {0,  0, -1};
    // constants
    const float G = 9.807f;
    const float _d2r = 3.14159265359f/180.0f;

    const float accRawScaling = 32767.5f; // =(2^16-1)/2 16 bit representation of acc value to cover +/- range
    const float gyroRawScaling = 32767.5f; // =(2^16-1)/2 16 bit representation of gyro value to cover +/- range
    const float magRawScaling = 32767.5f; // =(2^16-1)/2 16 bit representation of gyro value to cover +/- range

    const float _magScale = 4912.0f / magRawScaling; // micro Tesla, measurement range is +/- 4912 uT.

    const uint8_t ICM20948_WHO_AM_I = 0xEA;

    // ICM20948 registers
    // User bank 0
    const uint8_t UB0_WHO_AM_I = 0x00;
		const uint8_t UB0_USER_CTRL = 0x03;
		const uint8_t UB0_USER_CTRL_I2C_MST_EN = 0x20;

    const uint8_t UB0_PWR_MGMNT_1 = 0x06;
    const uint8_t UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO = 0x01;
    const uint8_t UB0_PWR_MGMNT_1_DEV_RESET = 0x80;

    const uint8_t UB0_PWR_MGMNT_2 = 0x07;
    const uint8_t UB0_PWR_MGMNT_2_SEN_ENABLE = 0x00;

    const uint8_t UB0_INT_PIN_CFG = 0x0F;
    const uint8_t UB0_INT_PIN_CFG_HIGH_50US = 0x00;

    const uint8_t UB0_INT_ENABLE_1 = 0x11;
    const uint8_t UB0_INT_ENABLE_1_RAW_RDY_EN = 0x01;
    const uint8_t UB0_INT_ENABLE_1_DIS = 0x00;


    const uint8_t UB0_ACCEL_XOUT_H = 0x2D;

    const uint8_t UB0_EXT_SLV_SENS_DATA_00 = 0x3B;

    // User bank 2
    const uint8_t UB2_GYRO_SMPLRT_DIV = 0x00;

    const uint8_t UB2_GYRO_CONFIG_1 = 0x01;
    const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_250DPS = 0x00;
    const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_500DPS = 0x02;
    const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_1000DPS = 0x04;
    const uint8_t UB2_GYRO_CONFIG_1_FS_SEL_2000DPS = 0x06;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_12106HZ = 0x00;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_197HZ = 0x00 | 0x01;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_152HZ = 0b00001000 | 0x01;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_120HZ = 0b00010000 | 0x01;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_51HZ  = 0b00011000 | 0x01;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_24HZ  = 0b00100000 | 0x01;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_12HZ  = 0b00101000 | 0x01;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_6HZ   = 0b00110000 | 0x01;
    const uint8_t UB2_GYRO_CONFIG_1_DLPFCFG_361HZ = 0b00111000 | 0x01;

    const uint8_t UB2_ACCEL_SMPLRT_DIV_1 = 0x10;
    const uint8_t UB2_ACCEL_SMPLRT_DIV_2 = 0x11;

    const uint8_t UB2_ACCEL_CONFIG = 0x14;
    const uint8_t UB2_ACCEL_CONFIG_FS_SEL_2G = 0x00;
    const uint8_t UB2_ACCEL_CONFIG_FS_SEL_4G = 0x02;
    const uint8_t UB2_ACCEL_CONFIG_FS_SEL_8G = 0x04;
    const uint8_t UB2_ACCEL_CONFIG_FS_SEL_16G = 0x06;
    const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_1209HZ = 0x00;
    const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_246HZ = 0x00 | 0x01;
    const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_111HZ = 0b00010000 | 0x01;
    const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_50HZ  = 0b00011000 | 0x01;
    const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_24HZ  = 0b00100000 | 0x01;
    const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_12HZ  = 0b00101000 | 0x01;
    const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_6HZ   = 0b00110000 | 0x01;
    const uint8_t UB2_ACCEL_CONFIG_DLPFCFG_473HZ = 0b00111000 | 0x01;

    // User bank 3
    const uint8_t UB3_I2C_MST_CTRL = 0x01;
    const uint8_t UB3_I2C_MST_CTRL_CLK_400KHZ = 0x07; // Gives 345.6kHz and is recommended to achieve max 400kHz

    const uint8_t UB3_I2C_SLV0_ADDR = 0x03;
    const uint8_t UB3_I2C_SLV0_ADDR_READ_FLAG = 0x80;

    const uint8_t UB3_I2C_SLV0_REG = 0x04;

    const uint8_t UB3_I2C_SLV0_CTRL = 0x05;
    const uint8_t UB3_I2C_SLV0_CTRL_EN = 0x80;

    const uint8_t UB3_I2C_SLV0_DO = 0x06;

    // Common to all user banks
    const uint8_t REG_BANK_SEL = 0x7F;
    const uint8_t REG_BANK_SEL_USER_BANK_0 = 0x00;
    const uint8_t REG_BANK_SEL_USER_BANK_1 = 0x10;
    const uint8_t REG_BANK_SEL_USER_BANK_2 = 0x20;
    const uint8_t REG_BANK_SEL_USER_BANK_3 = 0x30;

    // Magnetometer constants
		const uint8_t MAG_AK09916_I2C_ADDR = 0x0C;
		const uint16_t MAG_AK09916_WHO_AM_I = 0x4809;
		const uint8_t MAG_DATA_LENGTH = 8; // Bytes

		// Magnetometer (AK09916) registers
		const uint8_t MAG_WHO_AM_I = 0x00;

		const uint8_t MAG_HXL = 0x11;

		const uint8_t MAG_CNTL2 = 0x31;
		const uint8_t MAG_CNTL2_POWER_DOWN = 0x00;
		const uint8_t MAG_CNTL2_MODE_10HZ = 0x02;
		const uint8_t MAG_CNTL2_MODE_50HZ = 0x06;
		const uint8_t MAG_CNTL2_MODE_100HZ = 0x08;

		const uint8_t MAG_CNTL3 = 0x32;
		const uint8_t MAG_CNTL3_RESET = 0x01;

		// private functions
		int enableI2cMaster();
		int selectAutoClockSource();
		int enableAccelGyro();
		int reset();
		int changeUserBank(UserBank userBank);
		int changeUserBank(UserBank userBank, bool force);
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int writeMagRegister(uint8_t subAddress, uint8_t data);
    int readMagRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
    int whoAmIMag();
    int powerDownMag();
    int resetMag();
};

#endif