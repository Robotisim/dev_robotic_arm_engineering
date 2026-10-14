
#include <cassert>
#include <iostream>
#include "../../src/main.cpp"
void serial(const std::string&s){for(char c:s)Serial.input.push_back(c);}
void tick(unsigned ms=20){clockMs+=ms;loop();}
int main(){
 setup();assert(!outputsArmed&&!driverFault&&writes.size()==16);
 for(auto [c,on,off]:writes)assert(off==4096);
 writes.clear();tick();assert(writes.empty());
 const char* valid="CMD 0.2 -0.3839724 -0.9250245 -0.3665191 0";
 command_udp.input.push_back(valid);tick();assert(receivedCommandCount==0&&writes.empty());
 serial(std::string(40,'x')+"ARM\n");tick();tick();assert(!outputsArmed);
 serial("ARM\n");tick();assert(outputsArmed&&!commandHold&&!writes.empty());
 command_udp.input.push_back("CMD nan 0 -0.5 0 0");tick();assert(receivedCommandCount==0);
 command_udp.input.push_back(std::string(valid)+std::string(200,' ')+"junk");tick();assert(receivedCommandCount==0);
 command_udp.input.push_back(valid);tick();assert(receivedCommandCount==1&&targetAngleDeg[0]>10);
 tick(2100);assert(outputsArmed&&commandHold&&targetAngleDeg[0]==currentAngleDeg[0]);
 command_udp.input.push_back(valid);tick();assert(receivedCommandCount==1&&commandHold);
 serial("ARM\n");tick();assert(!commandHold);
 command_udp.input.push_back(valid);tick();assert(receivedCommandCount==2);
 serial("HOLD\n");tick();assert(commandHold&&targetAngleDeg[0]==currentAngleDeg[0]);
 serial("ARM\n");tick();failWrite=true;tick();assert(driverFault&&!outputsArmed&&commandHold);
 failWrite=false;auto before=writes.size();serial("ARM\n");tick();assert(!outputsArmed&&writes.size()==before);
 prefsOpen=true;prefsCount=6;assert(!loadJointSoftLimitsFromNvs());prefsCount=5;assert(loadJointSoftLimitsFromNvs());
 std::cout<<"PASS: boot outputs off, explicit ARM, disarmed UDP rejection, malformed packet rejection, timeout hold latch, HOLD/ARM, I2C fault latch, NVS joint count\n";
}
