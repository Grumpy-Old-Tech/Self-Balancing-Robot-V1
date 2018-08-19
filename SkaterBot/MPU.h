//
// Mpu Class
//

#ifndef MPU_H
#define MPU_H

#include "Arduino.h"
#include <Wire.h> 

#define MPU_REG_CONFIG           0x1A
#define MPU_REG_GYRO_CONFIG      0x1B
#define MPU_REG_ACCEL_CONFIG     0x1C

#define MPU_REG_ACCEL_XOUT_H     0x3B
#define MPU_REG_ACCEL_XOUT_L     0x3C
#define MPU_REG_ACCEL_YOUT_H     0x3D
#define MPU_REG_ACCEL_YOUT_L     0x3E
#define MPU_REG_ACCEL_ZOUT_H     0x3F
#define MPU_REG_ACCEL_ZOUT_L     0x40
#define MPU_REG_GYRO_XOUT_H      0x43
#define MPU_REG_GYRO_XOUT_L      0x44
#define MPU_REG_GYRO_YOUT_H      0x45
#define MPU_REG_GYRO_YOUT_L      0x46
#define MPU_REG_GYRO_ZOUT_H      0x47
#define MPU_REG_GYRO_ZOUT_L      0x48

#define MPU_REG_PWR_MGMT_1       0x6B

#define RAD2DEGREE               57.296;

class MPU
{
  public:
    MPU(int address);
    void initialise();
    void getGyroCalibrationValues();
    float getAccelAngle();
    void setTiltAngle(float angle);
    float getTiltAngle();

  private:
    int   accelCalibrationValue = -187;   // Enter the accelerometer calibration value
    int   busAddress;
    long  gyroYawCalibrationValue;
    long  gyroPitchCalibrationValue;
    float tiltAngle;

  private:
    void reset();
    void wakeup();
    void setGyroFullScaleRange();
    void setAccelFullScaleRange();
    void setLowPassFilter();
    long getRotationX();
    long getRotationY();
    long getRotationZ();
    long getAccelerationX();
    long getAccelerationY();
    long getAccelerationZ();
};

#endif

