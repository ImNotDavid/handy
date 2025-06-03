#include "finger.h"
#include "config.h"
#include "imu.h"
#include <Adafruit_NeoPixel.h>

#define FAULTn  16
float config_list[][4] = FINGER_CONFIG;
const int fingersNo = sizeof(config_list) / sizeof(config_list[0]);
Finger fingers[fingersNo];
int delay_micros = 1000000 / FREQ;
int prev_micros = micros();
Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRBW + NEO_KHZ800);

int target_finger,haptic_value;
#define BUFFER_SIZE 32
char inputBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;
bool receiving = false;

void parseInput(const char* data) {
  // Expect format: "12,34"
  char* comma = strchr(data, ',');
  if (comma != NULL) {
    *comma = '\0';  // Split the string at the comma
    target_finger = atoi(data);
    haptic_value = atoi(comma + 1);

    //Serial.print("Parsed: ");
    //Serial.print(target_finger);
    //Serial.print(" and ");
    //Serial.println(haptic_value);
    fingers[target_finger].setFeedback(static_cast<uint8_t>(haptic_value));
  } else {
    Serial.println("Error: no comma found.");
  }
}


void setup()
{
  pinMode(LED_ENABLE,OUTPUT);
  digitalWrite(LED_ENABLE,HIGH);
  led.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  led.setBrightness(BRIGHTNESS);
  led.setPixelColor(0, led.Color(194, 24, 7));
  led.show();            


  
  Wire.begin(SDA1, SCL1);
  Serial.begin(115200);
  Serial.println("\nHandy Test");
  setupIMU();

  // Setup fingers
  for (int i = 0; i < fingersNo; i++)
  {
    fingers[i] = Finger(static_cast<int>(config_list[i][0]),config_list[i][1],config_list[i][2],static_cast<bool>(config_list[i][3]));
  }

  for (int i = 0; i < fingersNo; i++)
  {
    fingers[i].initHaptics();
  }

  fingers[2].haptic_motor.setSpeed(0);
  led.setPixelColor(0, led.Color(147, 112, 219));
  led.show();    
}

void loop()
{
  int current = micros();
  if (current > (prev_micros + delay_micros))
  {
    getOrientation();
    float angles[10] = { fingers[0].readAngle(0), fingers[0].readAngle(1), fingers[0].readAngle(2), 
                        0.0, fingers[1].readAngle(0), fingers[1].readAngle(1), fingers[1].readAngle(2), 
                        fingers[2].readAngle(0), fingers[2].readAngle(1), fingers[2].readAngle(2)};
    Serial.write(0xAA);
    for(int i = 0; i < 10; i++){
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
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '<') {
      receiving = true;
      bufferIndex = 0;
    } else if (inChar == '>') {
      receiving = false;
      inputBuffer[bufferIndex] = '\0';  // Null-terminate the string
      parseInput(inputBuffer);
    } else if (receiving && bufferIndex < BUFFER_SIZE - 1) {
      inputBuffer[bufferIndex++] = inChar;
    }
  }
}



