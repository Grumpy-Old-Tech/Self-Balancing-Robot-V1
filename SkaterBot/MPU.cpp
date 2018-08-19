//
// MPU Class
//

#include "Arduino.h"
#include "MPU.h"

MPU::MPU(int address) {

  busAddress = address; 

  Wire.begin();   //Start the I2C bus as master
  TWBR = 12;      //Set the I2C clock speed to 400kHz
}

void MPU::initialise() {

  //reset();
  wakeup();
  setGyroFullScaleRange();
  setAccelFullScaleRange();
  setLowPassFilter();
}

void MPU::getGyroCalibrationValues() {

  int loopCounter;
  
  for (loopCounter = 0; loopCounter < 500; loopCounter++) {

    gyroYawCalibrationValue += getRotationX();
    gyroPitchCalibrationValue += getRotationY();
    delayMicroseconds(4000);  
  }
  
  gyroYawCalibrationValue /= 500;
  gyroPitchCalibrationValue /= 500;
}

float MPU::getAccelAngle() {
  
  long accelRawData;
  float accelAngle;

  //Get the Accel Angle
  accelRawData = getAccelerationZ() + accelCalibrationValue;
  constrain(accelRawData, -8200, 8200);
  accelAngle = asin((float)accelRawData / 8200.0) * RAD2DEGREE;
  return accelAngle;
}

void MPU::setTiltAngle(float angle) {

  tiltAngle = angle;
}

float MPU::getTiltAngle() {

  long gyroYawRawData;
  long gyroPitchRawData;
  long accelRawData;
  float accelAngle;

  //Get the Accel Angle
  accelRawData = getAccelerationZ() + accelCalibrationValue;
  constrain(accelRawData, -8200, 8200);
  accelAngle = asin((float)accelRawData / 8200.0) * RAD2DEGREE;

  gyroYawRawData = getRotationX();
  gyroPitchRawData = getRotationY() - gyroPitchCalibrationValue;
  tiltAngle += gyroPitchRawData * 0.000031;  // update with this loops travelled value

  // Compensate for mounting try -0.0000003 or 0.0000003 if required
  gyroYawRawData -= gyroYawCalibrationValue;
  //tiltAngle -= gyroYawRawData * 0.0000003;  
  
  // Correct the drift of the gyro angle with the accelerometer angle
  tiltAngle = tiltAngle * 0.9996 + accelAngle * 0.0004; 
  return tiltAngle;
}

void MPU::reset() {
  
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_PWR_MGMT_1);
  Wire.write(0b10000000);
  Wire.endTransmission();
}

void MPU::wakeup() {
  
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_PWR_MGMT_1);
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void MPU::setGyroFullScaleRange() {
  
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_GYRO_CONFIG);
  Wire.write(0b00000000);             // +/- 250 derees per second
  Wire.endTransmission();
}

void MPU::setAccelFullScaleRange() {
  
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_ACCEL_CONFIG);
  Wire.write(0b00001000);             // +/- 4G
  Wire.endTransmission();
}

void MPU::setLowPassFilter() {
  
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_CONFIG);
  Wire.write(0b00000011);             // 42/44 Hz
  Wire.endTransmission();
}

long MPU::getRotationX() {

  long value;
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(busAddress, 2);
  value = Wire.read() << 8 | Wire.read();
  return value;
}

long MPU::getRotationY() {

  long value;
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_GYRO_YOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(busAddress, 2);
  value = Wire.read() << 8 | Wire.read();
  return value;
}

long MPU::getRotationZ() {

  long value;
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_GYRO_ZOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(busAddress, 2);
  value = Wire.read() << 8 | Wire.read();
  return value;
}

long MPU::getAccelerationX() {

  long value;
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(busAddress, 2);
  value = Wire.read() << 8 | Wire.read();
  return value;
}

long MPU::getAccelerationY() {

  long value;
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_ACCEL_YOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(busAddress, 2);
  value = Wire.read() << 8 | Wire.read();
  return value;
}

long MPU::getAccelerationZ() {

  long value;
  Wire.beginTransmission(busAddress);
  Wire.write(MPU_REG_ACCEL_ZOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(busAddress, 2);
  value = Wire.read() << 8 | Wire.read();
  return value;
}

