#include "finger.h"
#include "config.h"
#include "imu.h"
#include "usb.h"

int addrList[] = ADDR_LIST;
float offsetList[][2] = OFFSETS_LIST;
const int fingersNo = sizeof(addrList) / sizeof(int);
Finger fingers[fingersNo];
int delay_micros = 1000000 / FREQ;
int prev_micros = micros();

void setup()
{
  USB.PID(0x0001);
  USB.VID(0x1234);
  USB.manufacturerName("ICRS");
  USB.productName("Handy Board");
  USB.firmwareVersion(0);
  USB.begin();
  Wire.begin(SDA1, SCL1);
  Serial.begin(115200);
  Serial.println("\nHandy Test");
  setupIMU();

  // Setup fingers
  for (int i = 0; i < fingersNo; i++)
  {
    fingers[i] = Finger(addrList[i], offsetList[i][0], offsetList[i][1]);
  }
}

void loop()
{
  int current = micros();
  if (current > (prev_micros + delay_micros))
  {
    getOrientation();
    float angles[7] = {fingers[0].readAngle(0), fingers[0].readAngle(1), fingers[0].readAngle(2), 0.0, fingers[1].readAngle(0), fingers[1].readAngle(1), fingers[1].readAngle(2)};
    Serial.write(0xAA);
    for(int i = 0; i < 7; i++){
      byte* outBytes = (byte*) &angles[i];
      Serial.write(outBytes,sizeof(float));
    }
    float outValue = filter.getRoll(); 
    Serial.write((byte*)&outValue,sizeof(float));
    outValue = filter.getPitch(); 
    Serial.write((byte*)&outValue,sizeof(float));
    outValue = filter.getYaw(); 
    Serial.write((byte*)&outValue,sizeof(float));
    Serial.write(0x55);
    prev_micros = micros();
  }
  
}
