
#pragma once
#include "Arduino.h"
inline std::vector<std::tuple<uint8_t,uint16_t,uint16_t>> writes;
inline bool failWrite=false,beginOk=true;
struct Adafruit_PWMServoDriver {
 Adafruit_PWMServoDriver(int){}
 bool begin(){return beginOk;}
 int setPWM(uint8_t c,uint16_t a,uint16_t b){if(failWrite)return 1;writes.emplace_back(c,a,b);return 0;}
 void setPWMFreq(int){}
};
