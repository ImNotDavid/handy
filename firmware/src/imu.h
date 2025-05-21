#include "MPU9250_WE.h"
#define MPU9250_ADDR 0x68
MPU6500_WE myIMU = MPU6500_WE(MPU9250_ADDR);
#include <MadgwickAHRS.h>
#include "config.h"

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setupIMU(){
  if(!myIMU.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myIMU.autoOffsets();
  Serial.println("Done!");
  myIMU.enableGyrDLPF();
  myIMU.setGyrDLPF(MPU6500_DLPF_6);
  myIMU.setSampleRateDivider(99); 
  myIMU.setGyrRange(MPU6500_GYRO_RANGE_250);
  myIMU.setAccRange(MPU6500_ACC_RANGE_2G);
  filter.begin(FREQ);
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}

void getOrientation(){
    float ax, ay, az;
    float gx, gy, gz;
    float roll, pitch, heading;
    unsigned long microsNow;
    xyzFloat correctedGyr = myIMU.getGyrValues();
    xyzFloat correctedAcc = myIMU.getGValues();
    gx = correctedGyr.x;
    gy = correctedGyr.y;
    gz = correctedGyr.z;
    ax = correctedAcc.x;
    ay = correctedAcc.y;
    az = correctedAcc.z;
    filter.updateIMU(gx, gy, gz, ax, ay, az);

}
