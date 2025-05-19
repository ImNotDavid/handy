#include "finger.h"
#include "config.h"

int addrList[] = ADDR_LIST;
float offsetList[][2] = OFFSETS_LIST;
const int fingersNo = sizeof(addrList)/sizeof(int);
Finger fingers[fingersNo];

void setup() {
  
  
  Wire.begin(SDA1,SCL1);
  Serial.begin(115200);
  Serial.println("\nHandy Test");

  // Setup fingers
  for(int i=0;i<fingersNo;i++){
    fingers[i] = Finger(addrList[i],offsetList[i][0],offsetList[i][1]); 
  }
}



void loop() {
  Serial.print(fingers[0].readAngle(0));
  Serial.print("\t");
  Serial.println(fingers[0].readAngle(1));
  delay(1000/FREQ);
}

