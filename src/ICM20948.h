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
    enum DlpfBandwidth
    {
      DLPF_BANDWIDTH_184HZ,
      DLPF_BANDWIDTH_92HZ,
      DLPF_BANDWIDTH_41HZ,
      DLPF_BANDWIDTH_20HZ,
      DLPF_BANDWIDTH_10HZ,
      DLPF_BANDWIDTH_5HZ
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
  	ICM20948(TwoWire &bus, uint8_t address);
  	int begin();
    /*int setAccelRange(AccelRange range);
    int setGyroRange(GyroRange range);
    int setDlpfBandwidth(DlpfBandwidth bandwidth);
    int setSrd(uint8_t srd);
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
    float getTemperature_C();*/
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
    float _magScaleX, _magScaleY, _magScaleZ;
    const float _tempScale = 333.87f;
    const float _tempOffset = 21.0f;
    // configuration
    AccelRange _accelRange;
    GyroRange _gyroRange;
    DlpfBandwidth _bandwidth;
    uint8_t _srd;
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
    /* transform the accel and gyro axes to match the magnetometer axes */
    const int16_t tX[3] = {0,  1,  0}; 
    const int16_t tY[3] = {1,  0,  0};
    const int16_t tZ[3] = {0,  0, -1};
    // constants
    const float G = 9.807f;
    const float _d2r = 3.14159265359f/180.0f;
    // ICM20948 registers
    // User bank 0
    const uint8_t UB0_WHO_AM_I = 0x00;
		const uint8_t UB0_USER_CTRL = 0x03;
		const uint8_t UB0_USER_CTRL_I2C_MST_EN = 0x20;
    const uint8_t UB0_PWR_MGMNT_1 = 0x06;
    const uint8_t UB0_PWR_MGMNT_1_CLOCK_SEL_AUTO = 0x01;

    const uint8_t UB0_PWR_MGMNT_2 = 0x07;
    const uint8_t UB0_PWR_MGMNT_2_SEN_ENABLE = 0x00;

    // Common to all user banks
    const uint8_t REG_BANK_SEL = 0x7F;
    const uint8_t REG_BANK_SEL_USER_BANK_0 = 0x00;
    const uint8_t REG_BANK_SEL_USER_BANK_1 = 0x10;
    const uint8_t REG_BANK_SEL_USER_BANK_2 = 0x20;
    const uint8_t REG_BANK_SEL_USER_BANK_3 = 0x30;

		// private functions
    int writeRegister(uint8_t subAddress, uint8_t data);
    int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
    int whoAmI();
};

#endif