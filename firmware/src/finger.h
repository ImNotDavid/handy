#include <Arduino.h>
#include <AS5600.h>
#include <Wire.h>
#define MOTOR_ADDR 0x60
#define ENC_0_BUS 0
#define ENC_1_BUS 4
#define MOTOR_BUS 0
#define EMPTY_BUS 1

class Finger
{
private:
    float enc_0_offset, enc_1_offset;
    AS5600 encoder0;
    AS5600 encoder1;
    int multi_address;
    void TCA9548A(uint8_t bus)
    {
        Wire.beginTransmission(multi_address); 
        Wire.write(1 << bus);                  
        Wire.endTransmission();
    }

public:
    Finger(){};
    Finger(int address, float offset0, float offset1)
    {   
        enc_0_offset = offset0;
        enc_1_offset = offset1;
        multi_address = address;
        TCA9548A(ENC_0_BUS);
        encoder0.setOffset(offset0);
        encoder0.begin();
        encoder0.setDirection(1);
        TCA9548A(ENC_1_BUS);
        encoder1.begin();
        encoder1.setOffset(offset1);
        TCA9548A(EMPTY_BUS);
    }
    float readAngle(int enc_no)
    {   
        float angle;
        if (enc_no == 0)
        {    
            TCA9548A(ENC_0_BUS);
            angle = encoder0.readAngle() * AS5600_RAW_TO_DEGREES;
            TCA9548A(EMPTY_BUS);
            return angle;
        }
        else if(enc_no == 1)
        {
            TCA9548A(ENC_1_BUS);
            angle = encoder1.readAngle() * AS5600_RAW_TO_DEGREES;
            TCA9548A(EMPTY_BUS);
            return angle;
        }
        else{
            angle = 90.0 - encoder1.readAngle() * AS5600_RAW_TO_DEGREES;
            return angle;
        }
    }
};